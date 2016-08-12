#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



class Visualization:

    def __init__(self, frame_id):
        topic = frame_id + "/visualization_marker_array"
        print topic
        self.rviz_publisher = \
            rospy.Publisher(topic, MarkerArray,
                    queue_size = 100)



    def visualizePoints(self, frame_id, shape, action, ns, scale, color, points,
            erase):
        
        if erase:
            markers_erase = MarkerArray()
            m_erase = Marker()
            m_erase.action = 3
            m_erase.ns = ns
            markers_erase.markers.append(m_erase)
            self.rviz_publisher.publish(markers_erase)
        markers = MarkerArray()
        c = 0
        print "Vizualize in Rviz"
        print len(points)
        for i in range(0, len(points)):
            c +=1
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = rospy.Time()
            m.type = shape
            m.action = action
            m.id = c
            m.ns = ns
            m.scale.x  = scale
            m.scale.y  = scale
            m.color.a = color[0]
            m.color.r = color[1]
            m.color.g = color[2]
            m.color.b = color[3]
            p1 = Point()
            p1.x = points[i].x
            p1.y = points[i].y
            m.points.append(p1)
            markers.markers.append(m)
        self.rviz_publisher.publish(markers)


    def visualizeVertices(self, frame_id, shape, action, ns, scale, color,
            points, erase):
        if erase:
            markers_erase = MarkerArray()
            m_erase = Marker()
            m_erase.action = 3
            m_erase.ns = ns
            markers_erase.markers.append(m_erase)
            self.rviz_publisher.publish(markers_erase)
        markers = MarkerArray()
        c = 0
        print "Vizualize in Rviz"
        print len(points)
        for i in range(0, len(points)):
            c +=1
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = rospy.Time()
            m.type = shape
            m.action = action
            m.id = c
            m.ns = ns
            m.scale.x  = scale
            m.scale.y  = scale
            m.scale.z  = scale
            m.color.a = color[0]
            m.color.r = color[1]
            m.color.g = color[2]
            m.color.b = color[3]
            m.pose.position.x = points[i].x
            m.pose.position.y = points[i].y
            markers.markers.append(m)
        self.rviz_publisher.publish(markers)


