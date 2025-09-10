# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO12, EFinger, EControlMode,EHandType
import time

def main():
    hand = AgibotHandO12(hand_type=EHandType.RIGHT)
    
    # reset
    aim_positions =  [ 0 ,0 ,0 ,0, -0, 0, 0 ,-0 ,0 ,0 ,0, 0]
    hand.set_all_joint_angles(aim_positions)
    time.sleep(1)
    
    # FIST
    aim_positions =  [ 0.5, -0.2, 0.0, -1.2, 0.0, 1.35, 1.53, 0.0, 1.36, 1.82, 1.55, 1.54]
    hand.set_all_joint_angles(aim_positions)
    time.sleep(1)
    

    real_positions = hand.get_all_joint_angles()
    print("All joint positions: ", real_positions)


if __name__ == "__main__":
    main()