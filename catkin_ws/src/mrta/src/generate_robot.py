#!/usr/bin/python2.7

import sys
import rospy
from DataGenerator import DataGenerator

if __name__ == "__main__":
    
    if len(sys.argv) != 4:
        sys.exit()
    
    robot_id = sys.argv[1]
    
    map_size_x = rospy.get_param("/map_size_x")
    map_size_y = rospy.get_param("/map_size_y")
    
    dg = DataGenerator(map_size_x, map_size_y)
    robot = dg.generate_robot(robot_id)

    print "Robot generated: " + str(robot)
    robot.start_listener()    
