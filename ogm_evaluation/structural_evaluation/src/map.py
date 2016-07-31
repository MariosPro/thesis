#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid
import numpy as np
class Map:
    def __init__(self, ogm):
        self.ros_ogm = ogm
        self.height = ogm.info.height
        self.width = ogm.info.width
        self.origin['x'] = self.origin.position.x
        self.origin['y'] = self.origin.position.y
        self.resolution = ogm.info.resolution

        self.ogm = np.zeros((self.width, self.height), dtype =np.int)

        for x in range(0, self.width):
            for y in range(0, self.height):
                self.ogm[x][y] = self.data[x + data.info.width * y]

