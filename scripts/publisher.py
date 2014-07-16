#!/usr/bin/env python

"""
Created on Tue Jul 15 21:00:14 2014

@author: Marco Esposito
"""

from xml_planning_scene.XMLSceneParser import XMLSceneParser
from moveit_msgs.msg import PlanningSceneWorld
import rospy
import argparse
import sys
import os

rospy.init_node('PlanningScenePublisher')

parser = argparse.ArgumentParser(description="Publishes the MoveIt! planning scene contained in an XML file")
parser.add_argument('file',help='scene file path')
args = parser.parse_args(args=rospy.myargv(sys.argv)[1:])
    
s = XMLSceneParser.parse_scene_file(args.file)
edit_time = os.stat(args.file).st_mtime

p = rospy.Publisher('/planning_scene_world',PlanningSceneWorld)

r = rospy.Rate(1) # 1hz
try:
    while not rospy.is_shutdown():
        if os.stat(args.file).st_mtime != edit_time:
            s = XMLSceneParser.parse_scene_file(args.file)
            edit_time = os.stat(args.file).st_mtime
        p.publish(s)
        r.sleep()
except rospy.ROSInterruptException: pass
    
    