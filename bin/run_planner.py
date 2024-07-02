#!/usr/bin/env python

import sys
import rospy
from puzzlebot_assembly.hardware_planner import HardwarePlanner
from puzzlebot_assembly.robots import Robots, RobotParam
from puzzlebot_assembly.control import Controller, ControlParam
from puzzlebot_assembly.behavior_lib import BehaviorLib

if __name__ == "__main__":
    try:
        N = rospy.get_param("/robot_num")
        eth = 1.5e-3
        dt = 0.2
        r_param = RobotParam(L=5e-2, anchor_base_L=0.8e-2, anchor_L=1.7e-2)
        c_param = ControlParam(vmax=0.06, wmax=1.0,
                        uvmax=1.0, uwmax=5.0,
                        mpc_horizon=3, constr_horizon=3, eth=eth)
        c = Controller(N, dt, c_param)
        #  pool = Pool()
        #  rsys = Robots(N, c, pool, robot_param=r_param, eth=eth, pilot_ids=[])
        pilot_ids = ['first']
        rsys = Robots(N, c, robot_param=r_param, eth=eth, pilot_ids=['first'])
        hp = HardwarePlanner(N, c_param, c, rsys)
        hp.start()
    except rospy.ROSInterruptException:
        pass
