# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand Pro 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO12, EFinger, EControlMode
import time

def main():
    hand = AgibotHandO12()
        
    # set temperature report period for all fingers
    periods = [500] * 12
    hand.set_all_temperature_report_periods(periods)

    time.sleep(1)
    
    #  get temperature report for finger 8
    temp = hand.get_temperature_report(8)
    print(f"关节8温度: {temp}°C")

    # get temperature report for all fingers
    all_temps = hand.get_all_temperature_reports()
    print(f"所有关节温度: {all_temps}")


if __name__ == "__main__":
    main()