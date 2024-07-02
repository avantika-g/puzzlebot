#!/usr/bin/env python

import sys
import rospy
from puzzlebot_assembly.keyboard_ctrl import KeyboardCtrl

if __name__ == "__main__":
    try:
        N = int(sys.argv[1])
        hc = KeyboardCtrl(N, is_pwm=True)
        hc.start()
    except rospy.ROSInterruptException:
        pass
