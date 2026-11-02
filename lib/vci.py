# Copyright (c) 2021, Ciena Corporation.
# All rights reserved.
#
# SPDX-License-Identifier: LGPL-2.1-only

"""
A simple emulation of vci for unit testing: there is no VCI bus, so
connecting fails the way the real client does.
"""

# pylint: disable=too-few-public-methods ; reason: stand-ins for vci's API


# pylint: disable-next=redefined-builtin ; reason: vci's name for it
class Exception(Exception):
    """vci.Exception"""
    def __init__(self):
        super().__init__('no VCI bus in unit tests')


class Client:
    """vci.Client: connecting fails"""
    def __init__(self):
        raise Exception()

    def emit(self, _module, _notification, _data):
        """emit a notification"""

    def state_by_model(self, _model):
        """the state of a model"""
        return {}


class Config:
    """vci.Config"""


class State:
    """vci.State"""


class Model:
    """vci.Model"""
    def __init__(self, name):
        self.name = name

    def config(self, _config):
        """attach the configuration handler"""
        return self

    def state(self, _state):
        """attach the state handler"""
        return self


class Component:
    """vci.Component"""
    def __init__(self, name):
        self.name = name

    def model(self, _model):
        """add a model"""
        return self

    def run(self):
        """start serving"""

    def wait(self):
        """wait until stopped"""
