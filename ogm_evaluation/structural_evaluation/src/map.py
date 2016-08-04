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

        self.ogm = np.zeros((self.width, self.height), dtype =np.int)

        for x in range(0, self.width):
            for y in range(0, self.height):
                self.ogm[x][y] = ogm.data[x + ogm.info.width * y]

