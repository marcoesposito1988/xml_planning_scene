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
parser.add_argument('frequency',help='update frequency in Hz (0 for no updating)',default=0,type=int)
args = parser.parse_args(args=rospy.myargv(sys.argv)[1:])
    
s = XMLSceneParser.parse_scene_file(args.file)
edit_time = os.stat(args.file).st_mtime

p = rospy.Publisher('/planning_scene_world',PlanningSceneWorld)

if args.frequency == 0:
    rospy.loginfo('publishing once scene file: '+args.file)
    s = XMLSceneParser.parse_scene_file(args.file)
    p.publish(s)
else:
    rospy.loginfo('publishing scene file: '+args.file+' with frequency: '+str(args.frequency))
    r = rospy.Rate(args.frequency)
    try:
        while not rospy.is_shutdown():
            if os.stat(args.file).st_mtime != edit_time:
                s = XMLSceneParser.parse_scene_file(args.file)
                edit_time = os.stat(args.file).st_mtime
            p.publish(s)
            r.sleep()
    except rospy.ROSInterruptException: pass

    