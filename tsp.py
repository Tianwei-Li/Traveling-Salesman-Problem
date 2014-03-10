#!/usr/bin/python

import sys
import os
import string
import networkx as nx
from sets import Set
import heapq


# data structure for tsp
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

# data structure for mst
class NODE2:
    id = 0
    distToT = 0.0
    def __init__(self, id):
        self.id = id

    def __del__(self):
        pass

    def __cmp__(self, other):
        return cmp(self.distToT, other.distToT)


def mst(nodes, G):
    if nodes[0] != 0:
        nodes.append(0)

    cost = 0.0

    nodeCnt = len(nodes)

    heap = []
    leftNodes = {}
    for i in range(nodeCnt):
        id = nodes[i]
        node = NODE2(id)
        if id == 0:
            node.distToT = float('-inf')
        else:
            node.distToT = float('inf')
        heapq.heappush(heap, node)
        leftNodes[id] = node
    
    while len(heap) != 0:
        u = heapq.heappop(heap)
        del leftNodes[u.id]
        if u.id != 0:
            cost += u.distToT
        
        u.distToT = float('-inf')
        needHeapify = False
        for idx in G.neighbors(u.id):
            if leftNodes.has_key(idx):
                v = leftNodes.get(idx)
                dist = G[u.id][v.id]['weight']
                if dist < v.distToT:
                    v.distToT = dist
                    needHeapify = True
        
        if needHeapify == True:
            heapq.heapify(heap)


    return cost


def main(argv):
    G = nx.read_gexf(argv[1], int)
    nodes = G.nodes()
    nodeCnt = len(nodes)

    # heuristic value for node 0
    heur = mst(nodes, G)
    
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
            heur = mst(leftNodes.keys(), G)

        u = heapq.heappop(heap)
        del leftNodes[u.id]
        path.append(u.id)
        needHeapify = False
        for idx in G.neighbors(u.id):
            if leftNodes.has_key(idx):
                v = leftNodes.get(idx)
                dist = G[u.id][v.id]['weight']
                if u.g + dist < v.g:
                    v.g = u.g + dist
                    v.f = v.g + heur
                    needHeapify = True
        
        if needHeapify == True:
            heapq.heapify(heap)


    #nodes = G.nodes()
    #edges = G.edges(data=True)
    Cost = 0.0
    print('Tour:'),
    for i in range(nodeCnt):
        #print(' '),
        print(path[i]),
        if i != 0:
            Cost += G[path[i-1]][path[i]]['weight']

    print('')
    Cost += G[path[nodeCnt-1]][0]['weight']
    
    print('Cost:'),
    print(Cost)

    
if __name__=='__main__':
    main(sys.argv)
