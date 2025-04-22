#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def publish_table_marker():
    rospy.init_node('tabletop_publisher')
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(0.5)  # Wait for publisher to connect
    
    # Create table top marker
    table_marker = Marker(
        type=Marker.CUBE,
        id=0,
        lifetime=rospy.Duration(),
        pose=Pose(Point(0, 0, -0.05), Quaternion(0, 0, 0, 1)),
        scale=Vector3(0.8, 0.8, 0.02),
        header=Header(frame_id='world'),
        color=ColorRGBA(0.54, 0.27, 0.07, 1.0),  # Brown color
    )
    
    # Create wooden texture through patterns
    grain_markers = []
    for i in range(10):
        grain = Marker(
            type=Marker.CUBE,
            id=i+1,
            lifetime=rospy.Duration(),
            pose=Pose(Point(0, (i*0.08) - 0.4, -0.049), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.8, 0.02, 0.005),
            header=Header(frame_id='world'),
            color=ColorRGBA(0.6, 0.3, 0.1, 0.3),  # Slightly lighter brown for grain
        )
        grain_markers.append(grain)
    
    # Add table legs
    leg_positions = [
        (0.35, 0.35, -0.3),
        (0.35, -0.35, -0.3),
        (-0.35, 0.35, -0.3),
        (-0.35, -0.35, -0.3)
    ]
    
    leg_markers = []
    for i, pos in enumerate(leg_positions):
        leg = Marker(
            type=Marker.CYLINDER,
            id=i+11,
            lifetime=rospy.Duration(),
            pose=Pose(Point(pos[0], pos[1], pos[2]), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.05, 0.05, 0.5),
            header=Header(frame_id='world'),
            color=ColorRGBA(0.54, 0.27, 0.07, 1.0),  # Brown color
        )
        leg_markers.append(leg)
    
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        # Publish table
        marker_pub.publish(table_marker)
        
        # Publish grain patterns
        for grain in grain_markers:
            marker_pub.publish(grain)
        
        # Publish table legs
        for leg in leg_markers:
            marker_pub.publish(leg)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_table_marker()
    except rospy.ROSInterruptException:
        pass