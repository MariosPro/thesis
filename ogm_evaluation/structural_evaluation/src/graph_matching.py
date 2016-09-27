#!/usr/bin/env python
import timeit
import math
import operator
import numpy as np
import graph_tool.all as gt
import cv2
import skimage.transform as tf
from skimage.measure import ransac
from skimage import io, exposure
from skimage import img_as_ubyte, img_as_float
from geometry_msgs.msg import Point
from alpha_star_search import BfsVisitor
from graph_utils import GraphUtils
from topological_graph import TopologicalGraph
from map import Map
import scipy.misc

def h(v, target, pos):
    return math.sqrt(sum((pos[v].a - pos[target].a) ** 2))


# Class for performing the topological Graph Matching
class GraphMatching:
    
    # Constructor
    def __init__(self, map1, map2):
        self.map1 = Map(map1, "visualization_map1")
        self.map2 = Map(map2, "visualization_map2")
        self.graph1 = TopologicalGraph()
        self.graph2 = TopologicalGraph()
 
    def extractTopologicalGraphs(self, parameters):
        start_time = timeit.default_timer()
        voronoiPoints1 = []
        verticesPoints1 = []
        voronoiPoints1, verticesPoints1 = self.graph1.extractTopologicalGraph(self.map1, parameters)
        elapsed = timeit.default_timer() - start_time
        print "Topological Graph 1 execution time (ms): ", elapsed * 1000

        voronoiPoints2 = []
        verticesPoints2 = []
        start_time = timeit.default_timer()
        voronoiPoints2, verticesPoints2 = self.graph2.extractTopologicalGraph(self.map2, parameters)
        elapsed = timeit.default_timer() - start_time
        print "Topological Graph 2 execution time (ms): ", elapsed * 1000
        return voronoiPoints1, verticesPoints1, voronoiPoints2, verticesPoints2

    def graphMatching(self):
        gt.remove_parallel_edges(self.graph1.graph)
        gt.remove_parallel_edges(self.graph2.graph)
 #        for v in self.graph1.graph.vertices():
            # print "vertice:", v, " neighbors=", v.out_degree()
        # for v in self.graph2.graph.vertices():
            # print "vertice:", v, " neighbors=", v.out_degree()

        self.graph1.graph = gt.GraphView(self.graph1.graph, vfilt=lambda v:
                                         v.out_degree() > 1)
        self.graph2.graph = gt.GraphView(self.graph2.graph, vfilt=lambda v:
                                         v.out_degree() > 1)
        gt.remove_parallel_edges(self.graph1.graph)
        gt.remove_parallel_edges(self.graph2.graph)
        print "Graph1 Vertices=", self.graph1.graph.num_vertices()
        print "Graph1 Edges= ", self.graph1.graph.num_edges()
        print "Graph2 Vertices=", self.graph2.graph.num_vertices()
        print "Graph2 Edges= ", self.graph2.graph.num_edges()
        subgraphPoses1 = self.graph1.graph.vp.pose.get_2d_array(range(0,2)).transpose()
        subgraphPoses2 = self.graph2.graph.vp.pose.get_2d_array(range(0, 2)).transpose()


   #      touched_v = self.graph1.graph.new_vp("bool", val=False)
        # touched_e = self.graph1.graph.new_ep("bool", val=False)
        vertexMapping = []
        # subgraph12 = gt.Graph(directed=False)
        candidateSubgraphs = []
        subgraphsVertices = []
        vM = []
        for v in self.graph1.graph.vertices():
            touched_v = self.graph1.graph.new_vp("bool", val=False)
            touched_e = self.graph1.graph.new_ep("bool", val=False)
            maxSubgraphVertices = 0
            subgraph12 = gt.Graph(directed=False)
            for visitLim in xrange(4, self.graph1.graph.num_vertices()):
                gt.bfs_search(self.graph1.graph, v,
                              BfsVisitor(touched_v, touched_e, visitLim))
                filteredGraph1 = gt.GraphView(self.graph1.graph, vfilt=touched_v,
                                            efilt=touched_e)
                subgraph1, vmap1, emap1 = GraphUtils.prune(filteredGraph1)

        #         print "subgraph1: V=", subgraph1.num_vertices(), "E=", \
                    # subgraph1.num_edges()

      #           print "subgraph1: " + GraphUtils.format_vlist(subgraph1)
                # print "S1 -> G1: " + GraphUtils.format_vmap(vmap1)
#             for v in subgraph1.vertices():
                    # print "Vertice", v, subgraph1.vp.pose[v].a
                    # for e in v.out_neighbours():
                        # print "edge ", v, " ", e

                vm = gt.subgraph_isomorphism(subgraph1,
                                             self.graph2.graph,
                                             vertex_label=(subgraph1.vp["exits"],
                                                         self.graph2.graph.vp["exits"]),
                                             induced=True)
#                 vm = gt.subgraph_isomorphism(subgraph1,
                                             # self.graph2.graph,
                                             # induced=True)

                if len(vm) > 0:
#                     print "found isomorphisms= ", len(vm), "start vertex: ", \
                        # i, "number of vertices: ", visitLim
                    if visitLim > maxSubgraphVertices:
                        vM = vm
                        subgraph12 = subgraph1.copy()
                        maxSubgraphVertices = visitLim
     #                    print "found max subgpraphs isomorphisms= ", len(vm), "start vertex: ", \
                        # i, "number of vertices: ", visitLim
            candidateSubgraphs.append(subgraph12)
            subgraphsVertices.append(subgraph12.num_vertices())
            vertexMapping.append(vM)
          
        g = self.graph2.graph.copy()
        vMasks = []
        eMasks = []
        maxSubgraph = max(enumerate(subgraphsVertices), key=operator.itemgetter(1))
        print maxSubgraph[0], maxSubgraph[1]
        vm = vertexMapping[maxSubgraph[0]]
        subgraph12 = candidateSubgraphs[maxSubgraph[0]]
        print "VertexMapping:", len(vm)

        for i in xrange(len(vm)):
            g.set_vertex_filter(None)
            g.set_edge_filter(None)
            vmask, emask = gt.mark_subgraph(g, subgraph12, vm[i])
            vMasks.append(vmask)
            eMasks.append(emask)
            g.set_vertex_filter(vmask)
            g.set_edge_filter(emask)
            assert gt.isomorphism(g, subgraph12)

            # estimate similarity transform of matched graphs using RANSAC
            subgraphPoses1 = subgraph12.vp.pose.get_2d_array(range(0,2)).transpose()
            subgraphPoses2 = g.vp.pose.get_2d_array(range(0, 2)).transpose()
            subgraphPoses1 += [self.map1.limits['min_x'], self.map1.limits['min_y']]
            subgraphPoses2 += [self.map2.limits['min_x'], self.map2.limits['min_y']]

            model, inliers = ransac((subgraphPoses1, subgraphPoses2),
                                           tf.SimilarityTransform, min_samples=3,
                                           residual_threshold=2)#, max_trials=100)

            outliers = inliers == False
            # warp image
            tform = tf.SimilarityTransform(scale=model.scale, 
                                           rotation=model.rotation,
                                           translation=model.translation)
           #  tform = tf.SimilarityTransform()
            # tform.estimate(subgraphPoses1, subgraphPoses2)
            print tform.scale, tform.rotation, tform.translation
            img_warped = tf.warp(self.map1.mapImage,
                                 tform.inverse,output_shape=self.map2.mapImage.shape, 
                                 mode='constant', cval=0)
            scipy.misc.imsave('/home/marios/Desktop/test2.png', img_warped)
            scipy.misc.imsave('/home/marios/Desktop/test1.png',
                    self.map1.mapImage)

            img1_warped = exposure.rescale_intensity(img_warped,
                    out_range='uint8')
            img1_warped = np.asarray(img1_warped,np.uint8)
            scipy.misc.imsave('/home/marios/Desktop/test2.png', img_warped)

            img2 = self.map2.mapImage
            blendedImage = cv2.addWeighted(img1_warped, 0.5, img2, 0.5, 0)

       #      blendedImage = exposure.rescale_intensity((img1_warped + img2), out_range='float')
            # blendedImage = img_as_ubyte(blendedImage)
            name = "/home/marios/Desktop/" + str(i) +".png"
            cv2.imwrite(name, blendedImage)

            # inlier_idxs = np.nonzero(inliers)[0]
            # print inliers
            # print inlier_idxs
        print "subgraph2: V=", g.num_vertices(), "E=", g.num_edges()
        print "subgraph1: V=", subgraph12.num_vertices(), "E=", \
            subgraph12.num_edges()

        subgraphPoses1 = subgraphPoses1[inliers]
        subgraphPoses2 = subgraphPoses2[inliers]
        print subgraphPoses1
        print subgraphPoses2

        return subgraphPoses1.tolist(), subgraphPoses2.tolist()

