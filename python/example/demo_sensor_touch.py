# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO12, EFinger
import matplotlib.pyplot as plt
from collections import deque
import time


def main():
    hand = AgibotHandO12()

    touch_data = hand.get_tactile_sensor_data(EFinger.THUMB)
    print(f"Thumb touch sensor data:")
    print(f"  Online state: {touch_data.online_state}")
    print(f"  Normal force: {touch_data.normal_force}")
    print(f"  Tangent force: {touch_data.tangent_force}")
    print(f"  Tangent force angle: {touch_data.tangent_force_angle}")
    print(f"  Channel values: {touch_data.channel_values}")
    print(f"  Capacitive approach: {touch_data.capacitive_approach}")

if __name__ == "__main__":
    main()