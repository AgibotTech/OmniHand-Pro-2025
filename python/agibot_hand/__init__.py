# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

from .agibot_hand_core import (
    AgibotHandO12,
    TouchSensorData,
    JointMotorErrorReport,
    MixCtrl
)
from enum import IntEnum

class EFinger(IntEnum):
    THUMB = 0x01
    INDEX = 0x02
    MIDDLE = 0x03
    RING = 0x04
    LITTLE = 0x05

class EControlMode(IntEnum):
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2
    POSITION_TORQUE = 3
    VELOCITY_TORQUE = 4
    POSITION_VELOCITY_TORQUE = 5
    UNKNOWN = 10

__all__ = [
    'AgibotHandO12',
    'EFinger',
    'EControlMode',
    'TouchSensorData',
    'JointMotorErrorReport',
    'MixCtrl'
]