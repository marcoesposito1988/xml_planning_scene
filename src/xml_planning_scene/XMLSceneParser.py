# -*- coding: utf-8 -*-
"""
Created on Wed Jul 16 11:49:06 2014

@author: Marco Esposito
"""

from moveit_msgs.msg import PlanningSceneWorld,CollisionObject
from shape_msgs.msg import SolidPrimitive,Plane,Mesh
from geometry_msgs.msg import Pose,Point,Quaternion
from std_msgs.msg import Header
import rospy
import rospkg
from lxml import etree
import os

class XMLSceneParser(object):
    SCHEME_FILE_PATH = 'data/scene.xsd'
    
    @staticmethod
    def parse_scene_file(path):
        xml = etree.parse(path)
        XMLSceneParser.validate_scene_document(xml)
        r = xml.getroot()
        return XMLSceneParser.handle_scene(r)
    
    @staticmethod
    def validate_scene_document(xml):
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('xml_planning_scene')
        scheme_path = os.path.join(base_path,XMLSceneParser.SCHEME_FILE_PATH)
        rospy.logdebug('opening scheme file '+scheme_path)
        xsd_doc = etree.parse(scheme_path)
        xsd = etree.XMLSchema(xsd_doc)
        xsd.validate(xml)
        if not xsd.error_log:
            rospy.loginfo('scene file validated without errors')
        else:
            rospy.logerr('scene file not valid!')
            rospy.logerr(xsd.error_log)
        
    @staticmethod
    def handle_scene(node):
        if node.tag != 'scene':
            raise ValueError
        
        s = PlanningSceneWorld()
        
        for c in node.getchildren():
            s.collision_objects.append(XMLSceneParser.handle_object(c))
        
        return s
    
    @staticmethod
    def handle_object(node):
        if node.tag != 'object':
            raise ValueError
            
        v = node.attrib
        
        h = Header(frame_id=v['frame_id'])
        co = CollisionObject(header=h,id=v['id'])
        
        for c in node.getchildren():
            if c.tag == 'cone':
                pr,po = XMLSceneParser.handle_cone(c)
                co.primitives.append(pr)
                co.primitive_poses.append(po)
            if c.tag == 'cylinder':
                pr,po = XMLSceneParser.handle_cylinder(c)
                co.primitives.append(pr)
                co.primitive_poses.append(po)
            if c.tag == 'sphere':
                pr,po = XMLSceneParser.handle_sphere(c)
                co.primitives.append(pr)
                co.primitive_poses.append(po)
            if c.tag == 'box':
                pr,po = XMLSceneParser.handle_box(c)
                co.primitives.append(pr)
                co.primitive_poses.append(po)
            if c.tag == 'plane':
                pr,po = XMLSceneParser.handle_plane(c)
                co.planes.append(pr)
                co.plane_poses.append(po)
            if c.tag == 'mesh':
                pr,po = XMLSceneParser.handle_mesh(c)
                co.meshes.append(pr)
                co.mesh_poses.append(po)
        
        return co
        
    @staticmethod
    def handle_cone(node):
        if node.tag != 'cone':
            raise ValueError
        
        v = node.attrib
        c = node.getchildren()
        
        p = SolidPrimitive(type=SolidPrimitive.CONE)
        p.dimensions = [0,0]
        p.dimensions[SolidPrimitive.CONE_RADIUS] = float(v['cone_radius'])
        p.dimensions[SolidPrimitive.CONE_HEIGHT] = float(v['cone_height'])
        return p,XMLSceneParser.handle_pose(c[0])
    
    @staticmethod
    def handle_cylinder(node):
        if node.tag != 'cylinder':
            raise ValueError
        
        v = node.attrib
        c = node.getchildren()
        
        p = SolidPrimitive(type=SolidPrimitive.CYLINDER)
        p.dimensions = [0,0]
        p.dimensions[SolidPrimitive.CYLINDER_RADIUS] = float(v['cylinder_radius'])
        p.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = float(v['cylinder_height'])
        return p,XMLSceneParser.handle_pose(c[0])
    
    @staticmethod
    def handle_sphere(node):
        if node.tag != 'sphere':
            raise ValueError
        
        v = node.attrib
        c = node.getchildren()
        
        p = SolidPrimitive(type=SolidPrimitive.SPHERE)
        p.dimensions = [0]
        p.dimensions[SolidPrimitive.SPHERE_RADIUS] = float(v['sphere_radius'])
        return p,XMLSceneParser.handle_pose(c[0])
    
    @staticmethod
    def handle_box(node):
        if node.tag != 'box':
            raise ValueError
        
        v = node.attrib
        c = node.getchildren()
        
        p = SolidPrimitive(type=SolidPrimitive.BOX)
        p.dimensions = [0,0,0]
        p.dimensions[SolidPrimitive.BOX_X] = float(v['box_x'])
        p.dimensions[SolidPrimitive.BOX_Y] = float(v['box_y'])
        p.dimensions[SolidPrimitive.BOX_Z] = float(v['box_z'])
        return p,XMLSceneParser.handle_pose(c[0])
    
    @staticmethod
    def handle_plane(node):
        if node.tag != 'plane':
            raise ValueError
        
        v = node.attrib
        c = node.getchildren()
        return (Plane([float(v['a']),float(v['b']),float(v['c']),float(v['d'])]),
                    XMLSceneParser.handle_pose(c[0]))
    
    @staticmethod
    def handle_mesh(node):
        raise NotImplementedError    
        # TODO: read mesh file and put it into a shape_msgs.Mesh
        if node.tag != 'mesh':
            raise ValueError
        
        v = node.attrib
        c = node.getchildren()
        return Mesh(),XMLSceneParser.handle_pose(c[0])
        
    @staticmethod
    def handle_pose(node):
        if node.tag != 'pose':
            raise ValueError
        
        c = node.getchildren()
        return (Pose(XMLSceneParser.handle_position(c[0]),
                     XMLSceneParser.handle_orientation(c[1])))
    
    @staticmethod
    def handle_position(node):
        if node.tag != 'position':
            raise ValueError
        
        v = node.attrib
        return Point(float(v['x']),float(v['y']),float(v['z']))
    
    @staticmethod
    def handle_orientation(node):
        if node.tag != 'orientation':
            raise ValueError
        
        v = node.attrib
        return Quaternion(float(v['x']),float(v['y']),float(v['z']),float(v['w']))
        
if __name__ == '__main__':
    examples_dir = '../data'
    example_scene = 'example.scene'
        
    s = XMLSceneParser.parse_scene_file(os.path.join(examples_dir,example_scene))
    
    print(s)