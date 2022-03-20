'''
Cable initialization with reeb graph. 
Modify the source code from:
Schulman, John, et al. "Tracking deformable objects with point clouds." 2013 IEEE International Conference on Robotics and Automation. IEEE, 2013.
'''
import networkx as nx, numpy as np, scipy.spatial.distance as ssd, scipy.interpolate as si
from collections import deque
import itertools
from numpy.random import rand
#networkx==1.9

class Rope_initialization():
    def __init__(self):
        self.MIN_SEG_LEN = 3
        self.MAX_COST = 50
        self.SKIP_COST = 1000 # per meter
        self.MAX_NEIGHBORS=5
        self.WEIGHTS = np.r_[
            3, # ang_a_disp: angle between end of source and displacement targ-source
            3, # ang_b_disp: angle between end of target and tisplacement source-targ
            30, # ang_a_b: angle between end of source and end of targ
            30,# fwddist_b_a: positive part of displacement `dot` source direction
            50,# backdist_b_a: negative part of ...
            50,# perpdist_b_a: displacement perpendicular to source direction
            30,# fwddist_a_b
            50,# fwddist_a_b
            50] # perpdist_a_b

    def find_path_through_point_cloud(self, xyzs, plotting = False):
        S = self.skeletonize_point_cloud(xyzs)
        segs = self.get_segments(S)
        S,segs = self.prune_skeleton(S, segs)
        segs3d = [np.array([S.node[i]["xyz"] for i in seg]) for seg in segs]
        C = self.make_cost_matrix(segs3d)
        PG = self.make_path_graph(C, [len(path) for path in segs3d])
        (score, nodes) = self.longest_path_through_segment_graph(PG)
        total_path = []
        for node in nodes[::2]:
            if node%2 == 0: total_path.extend(segs[node//2])
            else: total_path.extend(segs[node//2][::-1])        
        total_path_2d = self.remove_duplicate_rows(np.array([S.node[i]["xyz"] for i in total_path]))
        path = self.change_num_of_equal_distance_nodes(total_path_2d, 50)

        return path
        
    def prune_skeleton(self, S, segs):
        bad_segs = []
        bad_nodes = []
        for seg in segs:
            if len(seg) < self.MIN_SEG_LEN:# and 1 in [S.degree(node) for node in seg]:
                bad_segs.append(seg)
                for node in seg:
                    if S.degree(node) <= 2: bad_nodes.append(node)
        for seg in bad_segs: segs.remove(seg)
        for node in bad_nodes: S.remove_node(node)
        return S, segs
        
    ############## SKELETONIZATION ##############


    def points_to_graph(self, xyzs, max_dist):
        pdists = ssd.squareform(ssd.pdist(xyzs))
        G = nx.Graph()
        for (i_from, row) in enumerate(pdists):
            G.add_node(i_from, xyz = xyzs[i_from])
            to_inds = np.flatnonzero(row[:i_from] < max_dist)
            for i_to in to_inds:
                G.add_edge(i_from, i_to, length = pdists[i_from, i_to])
        return G

    def skeletonize_graph(self, G, resolution):
        partitions = []
        for SG in nx.connected_component_subgraphs(G):
            self.calc_distances(SG)
            partitions.extend(self.calc_reeb_partitions(SG,resolution))
        node2part = {}
        skel = nx.Graph()    
        for (i_part, part) in enumerate(partitions):
            xyzsum = np.zeros(2)#3
            for node in part:
                node2part[node] = i_part
                xyzsum += G.node[node]["xyz"]
            skel.add_node(i_part,xyz = xyzsum / len(part))
                
        for (node0, node1) in G.edges():
            if node2part[node0] != node2part[node1]:
                skel.add_edge(node2part[node0], node2part[node1])
            
        return skel

    def largest_connected_component(self, G):
        sgs = nx.connected_component_subgraphs(G)
        return sgs[0]

    def skeletonize_point_cloud(self, xyzs, point_conn_dist = .025, cluster_size = .04):
        G = self.points_to_graph(xyzs, point_conn_dist)
        S = self.skeletonize_graph(G, cluster_size)
        return S
            
    def calc_distances(self, G):
        node0 = G.nodes()[0]
        G.node[node0]["dist"] = 0
        frontier = deque([node0])        
        while len(frontier) > 0:
            node = frontier.popleft()
            node_dist = G.node[node]["dist"]
            for nei in G.neighbors_iter(node):
                nei_dist = G.node[nei].get("dist")
                if G.node[nei].get("dist") is None or G.edge[node][nei]["length"] + node_dist < nei_dist:
                    frontier.append(nei)
                    G.node[nei]["dist"] = G.edge[node][nei]["length"] + node_dist
            
    def calc_reeb_partitions(self, G, resolution):
        nodes = G.nodes()
        distances = np.array([G.node[node]["dist"] for node in nodes])
        bin_edges = np.arange(distances.min()-1e-5, distances.max()+1e-5, resolution)
        bin_inds = np.searchsorted(bin_edges, distances) - 1
        dist_partitions =  [[] for _ in range(bin_inds.max() + 1)]
        for (i_node,i_part) in enumerate(bin_inds):
            dist_partitions[i_part].append(nodes[i_node])
            
        reeb_partitions = []
        for part in dist_partitions:
            sg = G.subgraph(part)
            reeb_partitions.extend(nx.connected_components(sg))
        return reeb_partitions   
        
    ############### GENERATE PATHS FROM GRAPH ###########

    def get_segments(self, G):
        segments = []
        for SG in nx.connected_component_subgraphs(G):
            segments.extend(self.get_segments_connected(SG))
        return segments
    def get_segments_connected(self, G):
        segments = []
        for junc in self.get_junctions(G):
            for nei in G.neighbors(junc):
                seg = self.get_segment_from_junction(G,junc, nei)
                if seg[0] < seg[-1]:
                    segments.append(seg)
                if seg[0] == seg[-1] and seg[::-1] not in segments:
                    segments.append(seg)
                    
        return segments
    def get_junctions(self, G):
        return [node for (node,deg) in G.degree().items() if deg != 2]
    def get_segment_from_junction(self, G, junc, nei):
        seg = [junc, nei]
        while G.degree(seg[-1]) == 2:
            for node in G.neighbors(seg[-1]):
                if node != seg[-2]:
                    seg.append(node)
                    break
        return seg
        
        
    ############### LONGEST PATH STUFF ##############
    def calc_path_features(self, pos_a, dir_a, pos_b, dir_b):
        "dir_a points away from a"
        ang_a_disp = self.ang_between(dir_a, pos_a - pos_b)
        ang_b_disp = self.ang_between(dir_b, pos_b - pos_a)
        ang_a_b = self.ang_between(-dir_a, dir_b)
        fwddist_b_a = np.clip(np.dot(pos_b - pos_a, -dir_a),0,np.inf)
        backdist_b_a = -np.clip(np.dot(pos_b - pos_a, -dir_a),-np.inf,0)
        perpdist_b_a = np.sqrt(self.sqnorm(pos_b - pos_a) - np.dot(pos_b - pos_a, -dir_a)**2)
        fwddist_a_b = np.clip(np.dot(pos_a - pos_b, -dir_b),0,np.inf)
        backdist_a_b = -np.clip(np.dot(pos_a - pos_b, -dir_b),-np.inf,0)    
        perpdist_a_b = np.sqrt(self.sqnorm(pos_a - pos_b) - np.dot(pos_a - pos_b, -dir_b)**2)
        return np.array([
            ang_a_disp,
            ang_b_disp,
            ang_a_b,
            fwddist_b_a,
            backdist_b_a,
            perpdist_b_a,
            fwddist_a_b,
            backdist_a_b,
            perpdist_a_b])

    def sqnorm(self,x):
        return (x**2).sum()
    def ang_between(self,x,y):
        return np.arccos(np.dot(x,y)/np.linalg.norm(x)/np.linalg.norm(y))
    def normalized(self,x):
        return x / np.linalg.norm(x)
    def start_pos_dir(self,path):
        return path[0], self.normalized(path[min(len(path)-1,5)] - path[0])
    def end_pos_dir(self,path):
        return self.start_pos_dir(path[::-1])

    def estimate_end_directions(self,points, tol):
        points = np.asarray(points)
        n = len(points)
        deg = min(3, n - 1)
        u = np.arange(n)
        (tck, _) = si.splprep(points.T, s=tol**2*n, u=u, k=deg)    
        start_dir = np.array(si.splev(u[0],tck,der=1)).T
        end_dir = - np.array(si.splev(u[-1],tck,der=1)).T
        return self.normalized(start_dir), self.normalized(end_dir)

    def make_cost_matrix(self, paths):
        pos_and_dirs = []
        for path in paths:
            start_dir, end_dir = self.estimate_end_directions(path,.01)
            start_pos = path[0]
            end_pos = path[-1]
            pos_and_dirs.append((start_pos, start_dir))
            pos_and_dirs.append((end_pos, end_dir))

        N = len(paths)
        cost_matrix = np.zeros((2*N, 2*N))
        for (i,j) in itertools.combinations(range(2*N),2):
            if j==i+1 and i%2==0:
                cost_matrix[i,j] = cost_matrix[j,i] = np.inf
            else:            
                pdi = pos_and_dirs[i]
                pdj = pos_and_dirs[j]
                features = self.calc_path_features(pdi[0],pdi[1], pdj[0],pdj[1])
                features[np.isnan(features)] = 0
                cost_matrix[i,j] = cost_matrix[j,i] =  np.dot(self.WEIGHTS, features)

        return cost_matrix    

    def make_path_graph(self, cost_matrix, lengths):
        M,N = cost_matrix.shape
        assert M==N and M%2==0    

        path_graph = nx.Graph()
        for i in range(N):
            sortinds = cost_matrix[i].argsort()
            for j in sortinds[:self.MAX_NEIGHBORS]:
                if i!=j and cost_matrix[i,j] < self.MAX_COST:
                    path_graph.add_edge(i,j,weight = -cost_matrix[i,j])
        for i in range(0,N,2):
            if not path_graph.has_node(i): path_graph.add_node(i)
            if not path_graph.has_node(i+1): path_graph.add_node(i+1)
            path_graph.add_edge(i,i+1,weight=lengths[i//2]*self.SKIP_COST)
        return path_graph


    def longest_path_between(self, G,start,used,target):  
        
        opp = start - 1 if start%2==1 else start+1
        thislength, thispath = (G.edge[start][opp]["weight"],[start,opp])
        
        if opp == target:
            return (thislength, thispath)
        
        else:
            lengths_paths = []
            newused = used.union([start,opp])
            for nei in G.neighbors_iter(opp):
                if nei not in newused:
                    neilength,neipath = self.longest_path_between(G,nei,newused,target)
                    if neilength is not None:
                        lengths_paths.append(
                            (thislength+G.edge[opp][nei]["weight"]+neilength,thispath+neipath))

        if len(lengths_paths) > 0:
            return max(lengths_paths)    
        else: 
            return None,None
        
    def longest_path_from(self, G,start,used):  
        opp = start - 1 if start%2==1 else start+1
        thislength, thispath = (G.edge[start][opp]["weight"],[start,opp])
        lengths_paths = [(thislength, thispath)]
        newused = used.union([start,opp])
        for nei in G.neighbors_iter(opp):
            if nei not in newused:
                neilength,neipath = self.longest_path_from(G,nei,newused)
                lengths_paths.append(
                    (thislength+G.edge[opp][nei]["weight"]+neilength,thispath+neipath))
        return max(lengths_paths)    

    def longest_path_through_segment_graph(self, G):
        best_score = -np.inf
        best_nodes = []

        for i_start in G.nodes():
            (score,nodes) = self.longest_path_from(G, i_start, set([]))
            if score > best_score:
                best_score = score
                best_nodes = nodes
        return (best_score, best_nodes)


    def remove_duplicate_rows(self, mat):
        diffs = mat[1:] - mat[:-1]
        return mat[np.r_[True,(abs(diffs) >= 1e-5).any(axis=1)]]

    def change_num_of_equal_distance_nodes(self, nodes, out_N = 50):
        N = np.shape(nodes)[0]
        distances = np.zeros(N-1)
        for i in range(N-1):
            distances[i] = np.linalg.norm(nodes[i]-nodes[i+1])
        total_distance = np.sum(distances)
        equal_distance = total_distance/(out_N-1)
        new_nodes = np.zeros((out_N,2))
        i,j = 0,1
        start_point = nodes[0]
        new_nodes[0,:] = nodes[0,:]
        new_nodes[out_N-1,:] = nodes[N-1,:]
        need_distance = equal_distance
        # pdb.set_trace()
        while j < out_N-1:
            if np.linalg.norm(nodes[i+1]-start_point) > need_distance:
                new_nodes[j] = start_point + need_distance/np.linalg.norm(nodes[i+1]-start_point)*(nodes[i+1] - start_point)
                start_point = new_nodes[j]
                need_distance = equal_distance
                j += 1
            else:
                need_distance = need_distance - np.linalg.norm(nodes[i+1]-start_point)
                i += 1
                start_point = nodes[i]

        return new_nodes
