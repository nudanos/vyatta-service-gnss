#!/usr/bin/python3

# Copyright (c) 2021, Ciena Corporation. All rights reserved.
# Copyright (c) 2021, AT&T Intellectual Property. All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only

"""
Support for the ublox GNSS device in the S9500 30XS platform.
"""

# pylint: disable=protected-access ; reason: GPSUSB's UBX I/O is _gps_get/set

from enum import Enum, unique

import argparse
import array
import datetime
import json
import math
import os
import time
import syslog
import usb.core
import usb.util
import vci

from vyatta.gnss import GNSS
# pylint: disable-next=no-name-in-module ; reason: from vyatta-platform
from vyatta.platform.detect import PlatformError, detect

import timing_utility

from CPLD_utility import CPLDUtility
from const.const import Led
from protocol.lpc import LPC
from protocol.lpc import LPCDevType
from timing.gpsusb import GPSUSB


# pylint: disable-next=too-few-public-methods ; reason: plugin registration
class UbloxGNSS:
    """
    A helper class that registers the class that implements GNSS plugin.
    """
    def __init__(self):
        pass

    def probe(self, register):
        """
        If we are running on the S9500-30XS, register the plugin.
        """
        try:
            platform = detect()
            if platform.get_platform_string() == 'ufi.s9500-30xs':
                instance = _UbloxGNSS()
                register(instance)
                print('Adding UbloxGNSS instance', instance.get_instance())
        except PlatformError:
            pass


def gnss_dpll_state():
    """
    Check the state of the 1PPS and 10MHz timing signals sent
    to the IDT clock management chip from the ublox GNSS.

    If neither is valid, then the GNSS is free running (or
    no signal). If the only 10Mhz is valid, the GNSS is acquiring
    a lock. When both are valid, then the GNSS has a valid time fix.
    """
    timing_util = timing_utility.TimingUtility()

    one_pps = timing_util.get_dpll_input_status('GPS-1PPS')
    ten_mhz = timing_util.get_dpll_input_status('GPS-10MHz')
    return one_pps['valid_dpll1'], ten_mhz['valid_dpll2']


def gnss_antenna_state():
    """
    Fetch the antenna state from the CPLD register 0x1c.
    Two bits are used to indicate the short or open state.

    D[0] GNSS antenna shorted status
         0: GNSS antenna is shorted circuited
         1: GNSS antenna is not shorted circuited
    D[1] GNSS antenna open status
         0: GNSS antenna is open circuit
         1: GNSS antenna is not open circuit
    D[2..7] Reserved
    """
    lpc = LPC()
    GNSS_STATUS_REGISTER = 0x1c

    value = lpc.regGet(LPCDevType.CPLD_ON_MAIN_BOARD, GNSS_STATUS_REGISTER)
    is_shorted = value & 0x1 == 0x1
    is_open = value & 0x2 == 0x2
    return is_shorted, is_open


def gnss_antenna_status(is_shorted, is_open):
    """
    Return a valid antenna-status-enumeration from vyatta-service-gnss-v1
    """
    state_table = {
        (0, 0): "short",
        (0, 1): "unknown",
        (1, 0): "OK",
        (1, 1): "open",
    }
    return state_table[is_shorted, is_open]


def usb_write(usb_dev, cmd):
    """
    USB write to the Ublox GNSS device.
    """
    cfg = usb_dev.usb_dev.get_active_configuration()
    intf = cfg[(1, 0)]

    endpoint = usb.util.find_descriptor(
               intf,
               # match the first OUT endpoint
               custom_match=lambda e:
               usb.util.endpoint_direction(e.bEndpointAddress) ==
               usb.util.ENDPOINT_OUT)
    if endpoint is None:
        raise ValueError('EndpointAddress of USB device not found')

    endpoint.write(cmd)


@unique
class LedState(Enum):
    """
    Possible states for an LED
    """
    UNKNOWN = 0
    OFF = 1
    GREEN = 2
    BLINKING_GREEN = 3
    YELLOW = 4
    BLINKING_YELLOW = 5


class TimingState(Enum):
    """
    Possible states for the system timing core.
    """
    PHASE_LOCKED = 1
    FREQUENCY_LOCKED = 2
    HOLDOVER = 3
    FREE_RUN = 4


def dpll_get_timing_state():
    """
    Check and report the state of the system timing. If either
    DPLL is locked or in a pre-locked state, we report that.
    If we are in any of the other states, we choose Holdover
    or Free Run.

    DPLL1        DPLL2            DPLL1 or DPLL2
    PHASE_LOCK > FREQUENCY_LOCK > HOLDOVER       > FREE_RUN
    """
    def is_locked(dpll):
        if dpll['current'] is not None and \
           dpll['status']['operating_status'] in \
           ('Locked', 'Pre-locked', 'Pre-locked2'):
            return True
        return False

    def is_holdover(dpll):
        if dpll['status']['operating_status'] == 'Holdover':
            return True
        return False

    if not hasattr(dpll_get_timing_state, "previously_locked"):
        dpll_get_timing_state.previously_locked = False

    DPLL1 = 1
    DPLL2 = 2

    timing_util = timing_utility.TimingUtility()
    dpll1 = timing_util.get_dpll_status(DPLL1)
    dpll2 = timing_util.get_dpll_status(DPLL2)

    timing_state = TimingState.FREE_RUN

    if is_locked(dpll1):
        timing_state = TimingState.PHASE_LOCKED
        dpll_get_timing_state.previously_locked = True
    elif is_locked(dpll2):
        timing_state = TimingState.FREQUENCY_LOCKED
        dpll_get_timing_state.previously_locked = True
    elif (is_holdover(dpll1) or is_holdover(dpll2)) and \
            dpll_get_timing_state.previously_locked:
        timing_state = TimingState.HOLDOVER

    return timing_state


def u4(data):
    """
    Convert a little endian Ublox U4 type to a native integer
    """
    if len(data) != 4:
        raise ValueError('Need exactly 4 bytes')
    return int(data[0]) + \
        int(data[1]) * 256 + \
        int(data[2]) * 65536 + \
        int(data[3]) * 16777216


def i4(data):
    """
    Convert a little endian Ublox I4 type to a native integer
    """
    value = u4(data)
    if value > 0x7fffffff:
        value = value - 0x100000000
    return value


def checksum(command):
    """
    Calculate the UBX checksum for a given command
    """
    ck_a = 0
    ck_b = 0
    for b in command[2:]:
        ck_a += b
        ck_b += ck_a
    return [ck_a % 256, ck_b % 256]


def platform_delay():
    """
    It is typical for a platform to have some internal delay
    that is inherent to the hardware.
    """
    if os.path.exists('/run/vyatta/platform/ufi.s9500-30xs'):
        return 50

    return 0


def ubx_cfg_tp5(usb_dev, delay):
    """
    Set the antenna delay using the time pulse parameters.
    Must do this after configureUartTod or the configured
    delay will be overwritten.
    """
    # UBX-CFG-TP5
    # delay = 0 ns, RF group delay = 0 ns,
    # period = 1Hz, period (locked) = 10MHz,
    # pulseLen = 0, pulseLen (locked) = 50%,
    # active, lockGnssFreq, lockedOtherSet, isFreq,
    # alignToTow, polarity, gridUtcGnss = GPS, syncMode = 0
    cmd = array.array('B', [0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x01, 0x01,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                            0x00, 0x00, 0x80, 0x96, 0x98, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
                            0x00, 0x00, 0xEF, 0x00, 0x00, 0x00])

    delay += platform_delay()

    # Insert the antenna delay in little endian
    cmd[10] = delay & 0xff
    cmd[11] = (delay >> 8) & 0xff

    # Calculate and append the command checksum
    cmd += array.array('B', checksum(cmd))

    usb_dev._gps_set(cmd)


# pylint: disable-next=too-many-instance-attributes ; reason: device state
class _UbloxGNSS(GNSS):
    """
    ublox GNSS device support for the S9500 30XS platform.
    """
    MAX_RECORDS = 20

    def __init__(self):
        self.gpstime = None
        self.latitude = None
        self.longitude = None
        self.satellites = []
        self.enabled = True
        self.in_holdover = False
        self.holdover_start = None
        self.gnss_led = LedState.UNKNOWN
        self.sync_led = LedState.UNKNOWN
        self.antenna_status = None
        self.survey_status = None
        self.survey_observations = None
        self.survey_precision = None
        self.antenna_delay = 0

        syslog.openlog("vyatta-gnssd")

        # The GNSS might be stopped, ensure sure it is started
        # so that configuration will succeed.
        self.start()

    def start(self):
        """
        Start the ublox GNSS device.
        """
        def controlled_start(usb_dev):
            """
            Issue UBX-CFG-RST with resetMode = 0x9 (controlled start)
            """
            cmd = array.array('B', [0xb5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                    0x00, 0x00, 0x09, 0x00, 0x17, 0x76])
            usb_write(usb_dev, cmd)

        def configure_gnss(usb_dev):
            """
            Apply the default configuration from UfiSpace's BSP
            """

            def disable_GSV(usb_dev):
                """
                Disable GSV NMEA sentences
                """
                cmd = array.array('B', [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                        0xf0, 0x03, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00])
                cmd += array.array('B', checksum(cmd))
                usb_dev._gps_set(cmd)

            def disable_RMC(usb_dev):
                """
                Disable RMC NMEA sentences
                """
                cmd = array.array('B', [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00,
                                        0xf0, 0x04, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00])
                cmd += array.array('B', checksum(cmd))
                usb_dev._gps_set(cmd)

            usb_dev.enable()
            disable_GSV(usb_dev)
            usb_dev.configureUartTod()
            disable_RMC(usb_dev)

        def start_survey(usb_dev):
            """
            Start a survey with the UBX-CFG-TMODE2 command. The minimum
            duration for this survey is 5 minutes and the required
            accuracy for switching to fixed mode is 30 meters.
            """
            # UBX-CFG-TMODE2
            cmd = array.array('B', [0xb5, 0x62, 0x06, 0x3d, 0x1c, 0x00,
                                    0x01, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x2c, 0x01, 0x00, 0x00,  # 300s
                                    0x30, 0x75, 0x00, 0x00,  # 30m
                                    0x32, 0x0d])
            usb_dev._gps_set(cmd)

        usb_dev = GPSUSB()
        controlled_start(usb_dev)
        configure_gnss(usb_dev)
        ubx_cfg_tp5(usb_dev, self.antenna_delay)
        start_survey(usb_dev)

        self.enabled = True
        return True

    def stop(self):
        """
        Stop the ublox GNSS device.
        """
        def hardware_reset():
            """
            Issue UBX-CFG-RST with resetMode = 0x0 (hardware reset)
            """
            cmd = array.array('B', [0xb5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x0e, 0x64])
            usb_dev = GPSUSB()
            usb_write(usb_dev, cmd)

        def controlled_stop():
            """
            Issue UBX-CFG-RST with resetMode = 0x8 (controlled stop)
            """
            cmd = array.array('B', [0xb5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                    0x00, 0x00, 0x08, 0x00, 0x16, 0x74])
            usb_dev = GPSUSB()
            usb_write(usb_dev, cmd)

        # The hardware takes approximately 2 seconds to
        # return from hardware reset.
        RESET_TIME = 2

        hardware_reset()

        # Wait for the hardware to come back. After it
        # returns, stop all the GNSS tasks.
        while True:
            try:
                time.sleep(RESET_TIME)
                controlled_stop()
                break
            except usb.USBError:
                pass
            except ValueError:
                pass

        self.enabled = False
        return True

    def set_antenna_delay(self, delay):
        """
        Set the antenna delay using the TimePulse2 command.
        Must do this after configure_gnss or the configured
        delay will be overwritten.
        """
        usb_dev = GPSUSB()
        ubx_cfg_tp5(usb_dev, delay)

        self.antenna_delay = delay
        return True

    def get_antenna_delay(self):
        return self.antenna_delay

    def get_status(self):
        """
        Get the status of ublox GNSS device.
        """
        is_shorted, is_open = gnss_antenna_state()
        antenna_status = gnss_antenna_status(is_shorted, is_open)

        tracking_status = 'unknown'
        (one_pps, ten_mhz) = gnss_dpll_state()
        if one_pps and ten_mhz:
            tracking_status = 'tracking'
        elif ten_mhz:
            tracking_status = 'acquiring'

        if (one_pps or ten_mhz) and antenna_status == 'open':
            antenna_status = 'OK'

        status = {'instance-number': self.get_instance(),
                  'antenna-status': antenna_status,
                  'antenna-delay': self.antenna_delay,
                  'enabled': self.enabled,
                  'system': 'gps-system',
                  'tracking-status': tracking_status}

        # If the GNSS is stopped, we won't be able to
        # fetch any additional information.
        if self.enabled:
            self.fetch_gnss_data()

            if self.gpstime:
                status['time'] = self.gpstime

            if self.latitude:
                status['latitude'] = str(self.latitude)
            if self.longitude:
                status['longitude'] = str(self.longitude)

            if self.satellites:
                status['satellites-in-view'] = self.satellites

            if self.survey_status:
                status['survey-status'] = self.survey_status
                status['survey-observations'] = self.survey_observations
                if self.survey_precision:
                    status['survey-precision'] = self.survey_precision

        return status

    def fetch_ubx_nav_sat(self, usb_dev):
        """
        Issue UBX-NAV-SAT and reporting the visible satellites.
        See page 337 of the u-blox 8 / u-blox M8 Receiver description
        """
        self.satellites = []

        cmd = array.array('B', [0xb5, 0x62, 0x01, 0x35, 0x0, 0x0])
        cmd += array.array('B', checksum(cmd))

        try:
            response = usb_dev._gps_get(cmd)
        except usb.USBError:
            return

        # Skip 6 bytes of header and 8 bytes of payload
        for block_offset in range(14, len(response) - 2, 12):
            satellite = {}
            satellite['instance-number'] = len(self.satellites)

            # svId
            prn = response[block_offset + 1]
            satellite['PRN'] = prn

            # C/N0
            snr = int(response[block_offset + 2])
            if snr > 0:
                satellite['SNR'] = snr

            # Azimuth/Elevation
            satellite['azimuth'] = int(response[block_offset + 4])
            satellite['elevation'] = int(response[block_offset + 3])

            # svUsed from flags
            sv_used = (response[block_offset + 8] >> 3) & 0x1

            if sv_used:
                self.satellites.append(satellite)

    def fetch_ubx_nav_pvt(self, usb_dev):
        """
        Query time and position using UBX-NAV-PVT
        """
        self.latitude = None
        self.longitude = None
        self.gpstime = None

        cmd = array.array('B', [0xb5, 0x62, 0x01, 0x07, 0x0, 0x0])
        cmd += array.array('B', checksum(cmd))

        try:
            response = usb_dev._gps_get(cmd)
            if len(response) != 100:
                return
        except usb.USBError:
            return

        longitude = i4(response[30:34]) / 1.0e7
        latitude = i4(response[34:38]) / 1.0e7
        gnssFixOK = int(response[27]) & 0x1

        if gnssFixOK:
            self.latitude = latitude
            self.longitude = longitude

        # year (U2), month, day, hour, min, sec, valid flags
        year = response[10] + response[11] * 256
        month, day, hour, mins, secs, valid = (int(b) for b in response[12:18])

        if valid & 0x1 and valid & 0x2:    # validDate and validTime
            utc = datetime.datetime(year, month, day,
                                    hour=hour, minute=mins, second=secs,
                                    tzinfo=datetime.timezone.utc)
            self.gpstime = int(utc.timestamp())

    def fetch_svin(self, usb_dev):
        """
        Issue a UBX-CFG-SVIN to check the current status of survey-in.
        """
        cmd = array.array('B', [0xb5, 0x62, 0x0d, 0x04, 0x00, 0x00,
                                0x11, 0x40])

        try:
            response = usb_dev._gps_get(cmd)

            variance = u4(response[22:26])
            observations = u4(response[26:30])
            valid = int(response[30])
            active = int(response[31])
        except usb.USBError:
            valid = 0
            active = 0

        if active:
            self.survey_status = 'sampling'
        elif valid:
            self.survey_status = 'fixed-position'
        else:
            self.survey_status = None

        if active or valid:
            self.survey_observations = observations
        else:
            self.survey_observations = 0

        if self.survey_observations == 0:
            self.survey_precision = None
        else:
            self.survey_precision = math.sqrt(variance)

    def fetch_gnss_data(self):
        """
        Poll the GNSS data stream for NMEA sentences.
        """
        usb_dev = GPSUSB()
        self.fetch_ubx_nav_pvt(usb_dev)
        self.fetch_ubx_nav_sat(usb_dev)
        self.fetch_svin(usb_dev)

    def update_hardware_status(self):
        """
        Update the GPS and SYNC status LEDs.
        """
        is_shorted, is_open = gnss_antenna_state()

        state = self.gps_led_state(is_shorted, is_open)
        if self.gnss_led != state:
            update_led(Led.GPS, state)
            self.gnss_led = state

        state = self.sync_led_state()
        if self.sync_led != state:
            update_led(Led.SYNC, state)
            self.sync_led = state

        self.log_antenna_status(gnss_antenna_status(is_shorted, is_open))

    def gps_led_state(self, is_shorted, is_open):
        """
        The GPS LED state for the antenna and DPLL state:

        open=1, shorted=1                         => unconnected
        open=0, shorted=0                         => malfunction
        open=0, shorted=1,
        dpll_state = holdover|phase-lost|free-run => no-signal
        open=0, shorted=1,
        dpll_state = pre-locked1|pre-locked2      => acquiring
        open=0, shorted=1,
        dpll_state = pre-locked1|pre-locked2      => locked
        """
        if not self.enabled:
            return LedState.OFF
        if is_open == 0 and is_shorted == 0:
            return LedState.BLINKING_YELLOW
        if is_shorted != 1:
            return LedState.OFF

        (one_pps, ten_mhz) = gnss_dpll_state()
        if one_pps and ten_mhz:
            return LedState.GREEN
        if ten_mhz:
            return LedState.BLINKING_GREEN   # frequency input
        return LedState.YELLOW if is_open == 0 else LedState.OFF

    def sync_led_state(self):
        """
        The SYNC LED state for the timing state. This platform has a
        stratum 3e clock: precise enough to keep end-to-end PTP timing in
        holdover for at least 2 hours, after which the LED blinks.
        """
        in_spec_holdover_time = 7200    # seconds

        timing_state = dpll_get_timing_state()
        if timing_state != TimingState.HOLDOVER:
            self.in_holdover = False
        if timing_state == TimingState.PHASE_LOCKED:
            return LedState.GREEN
        if timing_state == TimingState.FREQUENCY_LOCKED:
            return LedState.BLINKING_GREEN
        if timing_state != TimingState.HOLDOVER:
            return LedState.OFF

        if not self.in_holdover:
            self.in_holdover = True
            self.holdover_start = time.time()
        if time.time() - self.holdover_start > in_spec_holdover_time:
            return LedState.BLINKING_YELLOW
        return LedState.YELLOW

    def log_antenna_status(self, antenna_status):
        """
        Log and notify an antenna status change.
        """
        if antenna_status == 'open':
            (one_pps, ten_mhz) = gnss_dpll_state()
            if one_pps or ten_mhz:
                antenna_status = 'OK'

        if antenna_status != self.antenna_status:
            instance_number = self.get_instance()
            syslog.syslog(syslog.LOG_WARNING,
                          f'GNSS {instance_number}: antenna status has '
                          f'changed to {antenna_status}')
            client = vci.Client()
            notification = {
                'antenna-status': antenna_status,
                'instance-number': instance_number
            }
            client.emit('vyatta-service-gnss-v1',
                        'antenna-status-update', notification)
            self.antenna_status = antenna_status


def update_led(led, state):
    """
    Set the given LED to the specific state
    """
    state_table = {
        LedState.OFF:
            (Led.STATUS_OFF, Led.COLOR_GREEN, Led.BLINK_STATUS_SOLID),
        LedState.GREEN:
            (Led.STATUS_ON, Led.COLOR_GREEN, Led.BLINK_STATUS_SOLID),
        LedState.BLINKING_GREEN:
            (Led.STATUS_ON, Led.COLOR_GREEN, Led.BLINK_STATUS_BLINKING),
        LedState.YELLOW:
            (Led.STATUS_ON, Led.COLOR_YELLOW, Led.BLINK_STATUS_SOLID),
        LedState.BLINKING_YELLOW:
            (Led.STATUS_ON, Led.COLOR_YELLOW, Led.BLINK_STATUS_BLINKING),
    }

    on_off, color, blink = state_table[state]

    cpld_util = CPLDUtility()
    cpld_util.set_led_control(led, on_off, color, blink)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--status", action='store_true',
                       help='Get the state of the ublox GNSS')
    group.add_argument("--start", action='store_true',
                       help='Start the ublox GNSS')
    group.add_argument("--stop", action='store_true',
                       help='Stop the ublox GNSS')
    group.add_argument("--update", action='store_true',
                       help='Update the hardware LEDs')
    args = parser.parse_args()

    gnss = _UbloxGNSS()
    gnss.set_instance(0)

    if args.status:
        print(json.dumps(gnss.get_status()))
    elif args.start:
        gnss.start()
    elif args.stop:
        gnss.stop()
    elif args.update:
        gnss.update_hardware_status()
    else:
        parser.print_help()
