# XML Planning Scene publisher
==============================

MoveIt! allows you to describe the environment around your robot by means of geometric primitives and polyonal meshes. While this is very useful for development setups where no sensor is used for collision avoidance, the lack of any other interface than the ROS API makes its use a slow and painful one. 

This tool allows you to define a planning scene in an XML file, with a convenient custom syntax. The file is monitored for changes after startup, so that any modification appears in Rviz right after saving. 

## Usage

An example launch file is provided. 

Alternatively, the publisher can be run directly:

    rosrun xml_planning_scene publisher.py path/to/file.scene

## Example XML Scene file

    <?xml version='1.0' encoding='utf-8'?>
    <scene 
    	xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    	xsi:noNamespaceSchemaLocation="scene.xsd">
    	
    	<object id="primitives" frame_id="world">
    		<box box_x="0.5" box_y="0.4" box_z="0.3">
    			<pose>
    				<position x="1" y="2" z="0" />
    				<orientation x="0" y="0" z="0" w="1" />
    			</pose>
    		</box>
    		
    		<cone cone_height="1" cone_radius="0.4">
    			<pose>
    				<position x="-5" y="2" z="0" />
    				<orientation x="0" y="0.707" z="0" w="0.707" />
    			</pose>
    		</cone>	
    	</object>
    	
    	<object id="plane" frame_id="map">
    		<plane a="1" b="2" c="3" d="4">
    			<pose>
    				<position x="1" y="2" z="-4" />
    				<orientation x="0" y="0" z="0" w="1" />
    			</pose>
    		</plane>
    	</object>
    </scene>
