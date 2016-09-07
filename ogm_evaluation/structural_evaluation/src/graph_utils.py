import graph_tool.all as gt


class GraphUtils:
    
    @staticmethod
    def prune(g):
        """ Returns a copy s of the filtered graph g, and a mapping (vmap,emap) from s to g """
        u = gt.Graph(g, prune=True)
        vmap = u.new_vertex_property("int")
        emap = u.new_edge_property("int")
        if u.num_vertices() > 0:
            vmap.a = g.vertex_index.copy("int").fa
        if u.num_edges() > 0:
            emap.a = g.edge_index.copy("int").fa

        return u, vmap, emap    
    
    @staticmethod
    def format_vlist(G):
        return ",".join(map(str,G.vertices()))

    @staticmethod
    def format_vmap(vmap):
        return ", ".join(["{}->{}".format(v,vmap[v]) for v in vmap.get_graph().vertices()])
