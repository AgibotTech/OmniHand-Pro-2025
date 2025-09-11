# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO12, EFinger, EControlMode
import time

def main():
    hand = AgibotHandO12()
    vendor_info = hand.get_vendor_info()
    print("Vendor Info:")
    print( vendor_info)

    device_info = hand.get_device_info() 
    print("Device Info:")
    print( device_info)


if __name__ == "__main__":
    main()