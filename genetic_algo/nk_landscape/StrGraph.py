import numpy as np
from numpy import array as narr
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

import pygraphviz as pgv
from itertools import combinations 

def hamming_distance(l1, l2):
    if len(l1) != len(l2):
        raise ValueError()
    dist = 0
    tmp_l = narr(l1) - narr(l2)
    dist = np.count_nonzero(tmp_l)
    return dist
    
def StrGraph(strlist, connect_d2=False):
    
    """
    Receives a list of strings(a list of objects of the same length)
    Returns a PyGraphViz Graph
    """
    
    strdict = dict(enumerate(strlist))
    
    G = pgv.AGraph()
    G.add_nodes_from(strdict)
    
    #comb =  
    for pair in combinations(strdict.keys(), 2):
    #print(pair)
        if hamming_distance(strdict[pair[0]], strdict[pair[1]])==1:
            G.add_edge(pair[0],pair[1], color='black')
        elif hamming_distance(strdict[pair[0]], strdict[pair[1]])==0:
            #G.remove_node(pair[1])
            G.add_edge(pair[0],pair[1], color='red')
            #tmp_node = G.get_node(pair[0])
            #tmp_node.attr['nreplica'] = str(int(tmp_node.attr['nreplica'])+1)
        elif hamming_distance(strdict[pair[0]], strdict[pair[1]])==2:
            #G.remove_node(pair[1])
            if connect_d2:
                G.add_edge(pair[0],pair[1], color='blue')
        
    G.layout()
    return G
    
#     You can visualize the graph by running the following lines
#     G.draw('testgraph.png')

#     plt.imshow(mpimg.imread('testgraph.png'))