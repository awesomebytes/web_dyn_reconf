#! /usr/bin/env python

import rospy
import dynamic_reconfigure.server
from web_dyn_reconf.cfg import TestConfig


def reconfigure(config, level):
    print config
    rospy.loginfo("Reconfigure req : %i %f %s %s" % (config['int_param'],
                                                     config['double_param'],
                                                     config['str_param'],
                                                     config['bool_param']))

    return config


if __name__ == '__main__':
    rospy.init_node("python_test_server")
    dynamic_reconfigure.server.Server(TestConfig, reconfigure)
    rospy.spin()
