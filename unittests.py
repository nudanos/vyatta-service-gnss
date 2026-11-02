#!/bin/python3

# Copyright (c) 2021, AT&T Intellectual Property.
# All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only


"""
Unit testing for vyatta-gnssd
"""

import subprocess
import time
import unittest


class TestGNSS(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # the daemon runs for the whole class; tearDownClass stops it
        # pylint: disable-next=consider-using-with ; reason: see above
        cls.daemon = subprocess.Popen(['sbin/vyatta-gnssd'],
                                      env={'PYTHONPATH': './lib'})
        time.sleep(1)

    def test_get_state(self):
        output = subprocess.check_output(
            ['bin/vyatta-gnss-util', '--get-state'])
        self.assertNotEqual(output, '')

    def test_do_stop(self):
        # check_output raises on failure
        subprocess.check_output(['bin/vyatta-gnss-util', '--stop-gnss'])

    def test_do_start(self):
        # check_output raises on failure
        subprocess.check_output(['bin/vyatta-gnss-util', '--start-gnss'])

    @classmethod
    def tearDownClass(cls):
        subprocess.check_output(['bin/vyatta-gnss-util', '--shutdown'])
        cls.daemon.wait(timeout=10)


class TestOpmode(unittest.TestCase):

    def test_do_show(self):
        output = subprocess.check_output(
            ['bin/vyatta-gnss-op', '--action=show'],
            env={'PYTHONPATH': './lib'})
        self.assertNotEqual(output, '')


if __name__ == '__main__':
    unittest.main()
