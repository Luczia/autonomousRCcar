#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from autonomous_vision.cfg import CfgrobotConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_sensibility}, {double_lspeed},{double_param}, {bool_param}""".format(**config))
    print(config["int_sensibility"])
    return config

if __name__ == "__main__":
    rospy.init_node("autonomous_vision_cfg", anonymous = True)

    srv = Server(CfgrobotConfig, callback)
    rospy.spin()
