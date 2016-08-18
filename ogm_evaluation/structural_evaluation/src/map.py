#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid
import numpy as np

class Map:
    def __init__(self, ogm, frame_id):
        self.ros_ogm = ogm
        self.height = ogm.info.height
        self.width = ogm.info.width
        self.origin = {}
        self.origin['x'] = ogm.info.origin.position.x
        self.origin['y'] = ogm.info.origin.position.y
        self.resolution = ogm.info.resolution
        self.frame_id = frame_id
        self.ogm = np.asarray(ogm.data).reshape(self.height, self.width)
        self.ogm = np.flipud(self.ogm)

        self.mapImage = np.zeros(self.ogm.shape, np.uint8)
        for i in xrange(self.ogm.shape[0]):
            for j in xrange(self.ogm.shape[1]):
                if self.ogm[i][j] == -1:
                    self.mapImage[i][j] = 127
                else:
                    self.mapImage[i][j] = (100 - self.ogm[i][j]) / 100 * 255


