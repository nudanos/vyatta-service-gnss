#!/usr/bin/python3

# Copyright (c) 2026, NuDanOS.
#
# SPDX-License-Identifier: LGPL-2.1-only

"""
Unit tests for the ublox GNSS plugin's UBX decoding. The platform modules
it imports (usb, the CPLD and timing utilities, vci, vyatta.platform) are
replaced by empty stand-ins; only the decoding is exercised.
"""

import array
import importlib.util
import os
import struct
import sys
import time
import types
import unittest

# pylint: disable=protected-access ; reason: testing the private device class


def _stub(name, **attrs):
    module = types.ModuleType(name)
    module.__dict__.update(attrs)
    sys.modules[name] = module
    return module


class _USBError(Exception):
    """stand-in for usb.USBError"""


_stub('usb', USBError=_USBError)
_stub('usb.core')
_stub('usb.util')
_stub('vci', Client=object)
_stub('vyatta.platform')
_stub('vyatta.platform.detect', PlatformError=Exception, detect=None)
_stub('timing_utility')
_stub('CPLD_utility', CPLDUtility=object)
_stub('const')
_stub('const.const', Led=types.SimpleNamespace())
_stub('protocol')
_stub('protocol.lpc', LPC=object, LPCDevType=object)
_stub('timing')
_stub('timing.gpsusb', GPSUSB=object)
sys.path.insert(0, 'lib')

_SPEC = importlib.util.spec_from_file_location(
    'UbloxGNSS', 'lib/vyatta/gnss-plugins/UbloxGNSS.py')
ublox = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(ublox)


def nav_pvt(lon, lat, fix_ok=True):
    """A 100-byte UBX-NAV-PVT response (6-byte header, 92-byte payload,
    checksum) for 2026-10-22 12:34:56 UTC at lon/lat in degrees."""
    payload = bytearray(92)
    struct.pack_into('<HBBBBBB', payload, 4, 2026, 10, 22, 12, 34, 56, 0x3)
    payload[21] = 0x1 if fix_ok else 0x0
    struct.pack_into('<ii', payload, 24, round(lon * 1e7), round(lat * 1e7))
    return array.array('B', b'\xb5\x62\x01\x07\x5c\x00' + payload + b'\0\0')


# pylint: disable-next=too-few-public-methods ; reason: GPSUSB's one call
class FakeUSB:
    """answers every UBX query with one canned response"""
    def __init__(self, response):
        self.response = response

    def _gps_get(self, _cmd):
        return self.response


class TestDecoding(unittest.TestCase):

    def setUp(self):
        self.tz = os.environ.get('TZ')
        os.environ['TZ'] = 'America/New_York'
        time.tzset()

    def tearDown(self):
        if self.tz is None:
            del os.environ['TZ']
        else:
            os.environ['TZ'] = self.tz
        time.tzset()

    def test_u4_i4(self):
        self.assertEqual(ublox.u4([1, 2, 3, 4]), 0x04030201)
        self.assertEqual(ublox.i4([0xff, 0xff, 0xff, 0xff]), -1)
        with self.assertRaises(ValueError):
            ublox.u4([1, 2, 3])

    def test_checksum(self):
        cmd = [0xb5, 0x62, 0x01, 0x07, 0x00, 0x00]
        self.assertEqual(ublox.checksum(cmd), [0x08, 0x19])

    def test_nav_pvt_position_and_time(self):
        gnss = object.__new__(ublox._UbloxGNSS)
        gnss.fetch_ubx_nav_pvt(FakeUSB(nav_pvt(lon=-0.1276, lat=51.5072)))
        self.assertAlmostEqual(gnss.latitude, 51.5072)
        self.assertAlmostEqual(gnss.longitude, -0.1276)
        self.assertEqual(gnss.gpstime, 1792672496)  # UTC, whatever the zone

    def test_nav_pvt_without_fix(self):
        gnss = object.__new__(ublox._UbloxGNSS)
        gnss.fetch_ubx_nav_pvt(FakeUSB(nav_pvt(1.0, 2.0, fix_ok=False)))
        self.assertIsNone(gnss.latitude)
        self.assertIsNone(gnss.longitude)


class TestLeds(unittest.TestCase):

    def setUp(self):
        self.gnss = object.__new__(ublox._UbloxGNSS)
        self.gnss.enabled = True
        self.gnss.in_holdover = False
        self.gnss.holdover_start = None
        self.saved = (ublox.gnss_dpll_state, ublox.dpll_get_timing_state)

    def tearDown(self):
        ublox.gnss_dpll_state, ublox.dpll_get_timing_state = self.saved

    def test_gps_led(self):
        led = ublox.LedState
        cases = [
            # (enabled, shorted, open, (1pps, 10MHz), LED)
            (False, 1, 0, (1, 1), led.OFF),
            (True, 0, 0, (1, 1), led.BLINKING_YELLOW),
            (True, 0, 1, (1, 1), led.OFF),
            (True, 1, 0, (1, 1), led.GREEN),
            (True, 1, 0, (0, 1), led.BLINKING_GREEN),
            (True, 1, 0, (0, 0), led.YELLOW),
            (True, 1, 1, (1, 1), led.GREEN),
            (True, 1, 1, (0, 1), led.BLINKING_GREEN),
            (True, 1, 1, (0, 0), led.OFF),
        ]
        for enabled, shorted, is_open, dpll, want in cases:
            self.gnss.enabled = enabled
            ublox.gnss_dpll_state = lambda dpll=dpll: dpll
            self.assertEqual(self.gnss.gps_led_state(shorted, is_open), want,
                             (enabled, shorted, is_open, dpll))

    def test_sync_led(self):
        led, timing = ublox.LedState, ublox.TimingState
        for state, want in [(timing.PHASE_LOCKED, led.GREEN),
                            (timing.FREQUENCY_LOCKED, led.BLINKING_GREEN),
                            (timing.HOLDOVER, led.YELLOW)]:
            ublox.dpll_get_timing_state = lambda state=state: state
            self.assertEqual(self.gnss.sync_led_state(), want, state)
        self.assertTrue(self.gnss.in_holdover)
        self.gnss.holdover_start -= 7201
        self.assertEqual(self.gnss.sync_led_state(), led.BLINKING_YELLOW)
        ublox.dpll_get_timing_state = lambda: timing.PHASE_LOCKED
        self.gnss.sync_led_state()
        self.assertFalse(self.gnss.in_holdover)


if __name__ == '__main__':
    unittest.main()
