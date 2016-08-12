#!/usr/bin/env python

# import rospy
# import sys
# from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from skimage.morphology import skeletonize
from skimage import draw
from visualization import Visualization
import numpy as np
import matplotlib.pyplot as plt

class VoronoiDiagram:
    
    def extractVoronoi(self, map):
        
        visualization = Visualization(map.frame_id)

        # self.image[map.ogm < 0] = 100
        width = map.ogm.shape[0]
        height = map.ogm.shape[1]
        binary = np.zeros(map.ogm.shape)

        binary[map.ogm <= 50.0] = 1
        print binary.shape
        # perform skeletonization
        skeleton = skeletonize(binary)
        print skeleton.shape
        voronoiPoints = []
        vizPoints = []
        for i in range(skeleton.shape[0]):
            for j in range(skeleton.shape[1]):
                if skeleton[i][j] == 1:
                    p = Point()
                    p.x = j
                    p.y = map.height- i
                    voronoiPoints.append(p)
                    vp = Point()
                    vp.x = j * map.resolution + map.origin['x']
                    vp.y = (map.height - i) * map.resolution + map.origin['y']
                    vizPoints.append(vp)

        print "Voronoi Diagram extracted"

        if map.frame_id == "visualization_map1":
            colors = [1.0, 1.0, 0.0, 0.0]
            ns = "map1/voronoiPoints"
        else:
            colors =[1.0, 0.0, 0.0, 1.0]
            ns = "map2/voronoiPoints"
        visualization.visualizePoints(map.frame_id,
                                8,
                                0,
                                ns,
                                0.02,
                                colors,
                                vizPoints,
                                True)

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

        return skeleton


