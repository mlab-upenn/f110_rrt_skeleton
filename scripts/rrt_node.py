#!/usr/bin/env python
import rospy
from rrt import RRT

def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()