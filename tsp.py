#!/usr/bin/python

import sys
import os
import string
import networkx as nx
from sets import Set
import heapq

class NODE:
    id = 0
    g = 0.0    # maintain the current shortest path from 0 to this node
    f = 0.0    # maintain the estimate of length of best path from s to t through this node
    def __init__(self, id): #constructor
        self.id = id

    def __del__(self):      #destructor
        pass

    #def setg(self, g):
    #    self.g = g

    #def setf(self, f):
    #    self.f = f

    def __cmp__(self, other):
        return cmp(self.f, other.f)


def mst(nodes):
    if nodes[0] != 0:
        nodes.append(0)
    #leftNodes.add(0)
    #weight = 0.0

    return 0.0


def main(argv):
    G = nx.read_gexf(argv[1], int)
    nodes = G.nodes()
    nodeCnt = len(nodes)

    # heuristic value for node 0
    heur = mst(nodes)
    
    # maintain the heap to find the next shortest node
    heap = []

    # hashmap to store the unvisited nodes
    leftNodes = {}
    
    for i in range(nodeCnt):
        node = NODE(i)
        if i == 0:
            node.g = 0
            node.f = heur
        else:
            temp = G[0][i]['weight']
            node.g = temp
            node.f = temp + heur

        heapq.heappush(heap, node)
        leftNodes[i] = node

    
     
    path = []
    while len(heap) != 0:
        
        if heap[0].id != 0:
            heur = mst(leftNodes.keys())

        u = heapq.heappop(heap)
        del leftNodes[u.id]
        path.append(u.id)
        for idx in G.neighbors(u.id):
            if leftNodes.has_key(idx):
                v = leftNodes.get(idx)
                dist = G[u.id][v.id]['weight']
                if u.g + dist < v.g:
                    v.g = u.g + dist
                    v.f = v.g + heur
                    heapq.heapify(heap)


    #nodes = G.nodes()
    #edges = G.edges(data=True)
    Cost = 0.0
    print('Tour:'),
    for i in range(nodeCnt):
        print(' '),
        print(path[i]),
        if i != 0:
            Cost += G[path[i-1]][path[i]]['weight']

    print('')
    Cost += G[path[nodeCnt-1]][0]['weight']
    
    print('Cost: '),
    print(Cost)
    #print(nodes)
    #print(edges)
    
    #G2 = nx.Graph()
    #G2.add_nodes_from([1,2,3])
    #G2.add_edge(1, 2)
    #G2.add_edge(1, 3)
    #G2[1][2]['weight'] = 1.2
    #G2[1][3]['weight'] = 1.3



if __name__=='__main__':
    main(sys.argv)
