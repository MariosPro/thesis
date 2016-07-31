#!/usr/bin/env python

# import rospy
# import sys
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import Point
from skimage.morphology import skeletonize
from skimage import draw
from visualization import Visualization
# import numpy as np
import matplotlib.pyplot as plt

class VoronoiDiagram:
    
    def extractVoronoi(self, map):
        # self.image[map.ogm < 0] = 100
        binary = map.ogm  <= 50.0
        # perform skeletonization
        skeleton = skeletonize(binary)
        voronoiPoints = []
        for i in range(skeleton.shape[0]):
            for j in range(skeleton.shape[1]):
                if skeleton[i][j] == 1:
                    p = Point()
                    p.x = i
                    p.y = j
                    voronoiPoints.append(p)
                    vizPoints.append(i * map.resolution + map.origin['x'], j *
                            map.resolution + map.origin['y'])

        print "Voronoi Diagram extracted"

        Visualization.visualize("visualization_map",
                8,
                2,
                "voronoi_points",
                0.02,
                [1.0, 1.0, 0.0, 0.0],
                vizPoints,
                )

        # display results
       #  fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4.5), sharex=True, sharey=True, subplot_kw={'adjustable':'box-forced'})

        # ax1.imshow(binary, cmap=plt.cm.gray)
        # ax1.axis('off')
        # ax1.set_title('original', fontsize=20)

        # ax2.imshow(skeleton, cmap=plt.cm.gray)
        # ax2.axis('off')
        # ax2.set_title('skeleton', fontsize=20)

        # fig.subplots_adjust(wspace=0.02, hspace=0.02, top=0.98,
                            # bottom=0.02, left=0.02, right=0.98)

        # plt.show()
        # plt.close()

        return voronoiPoints

    # def visualize(self, voronoiPoints):
        # markers_erase = MarkerArray()
        # m_erase = Marker()
        # m_erase.action = 3
        # m_erase.ns = "voronoi_diagram_connections"
        # markers_erase.markers.append(m_erase)
        # self.rviz_publisher.publish(markers_erase)

        # markers = MarkerArray()
        # c = 0
        # for i in range(0, len(voronoiPoints)):
            # c +=1
            # m = Marker()
            # m.header.frame_id = "map"
            # m.header.stamp = rospy.Time()
            # m.type = m.POINTS
            # m.action = m.ADD
            # m.id = c
            # m.ns = "voronoi_diagram_connections"
            # m.scale.x  = 0.02
            # m.scale.y  = 0.02
            # m.color.a = 1.0
            # m.color.r = 1.0
            # m.color.g = 0.0
            # m.color.b = 0.0
            # p1 = Point()
            # p1.x = voronoiPoints[i].y * self.resolution + self.origin.position.x
            # p1.y = (self.height - voronoiPoints[i].x) * self.resolution  + self.origin.position.y
            # m.points.append(p1)
            # markers.markers.append(m)
        # self.rviz_publisher.publish(markers)

