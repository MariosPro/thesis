#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


class Visualization:
    rviz_publisher = \
            rospy.Publisher('visualization_marker_array', MarkerArray,
                    queue_size = 1000)
    @staticmethod
    def visualize(frame, shape, action, ns, scale, color, points):
        markers_erase = MarkerArray()
        m_erase = Marker()
        m_erase.action = 3
        m_erase.ns = ns
        markers_erase.markers.append(m_erase)
        Visualization.rviz_publisher.publish(markers_erase)

        markers = MarkerArray()
        c = 0
        for i in range(0, len(points)):
            c +=1
            m = Marker()
            m.header.frame_id = frame
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
        Visualization.rviz_publisher.publish(markers)



