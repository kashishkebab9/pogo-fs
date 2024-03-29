import networkx as nx
import numpy as np
import math
import sys
from scikits.sparse.cholmod import cholesky

np.set_printoptions(threshold=np.inf)

pose_graph = nx.read_graphml("pose_graph.graphml")
num_nodes = nx.number_of_nodes(pose_graph)
H = np.zeros((num_nodes * 3, num_nodes * 3))
b = np.zeros((num_nodes * 3))

for edge in pose_graph.edges(data=True):

    edge_source, edge_target, edge_attr = edge

    x_i = edge_attr["x_i"]
    x_j = edge_attr["x_j"]   
    x_z = edge_attr["x_z"]   
    y_i = edge_attr["y_i"]   
    y_j = edge_attr["y_j"]   
    y_z = edge_attr["y_z"]   
    psi = edge_attr["psi"]   
    alpha = edge_attr["alpha"]   

    a_11 =  0
    a_12 =  0
    a_13 =  -x_j*math.sin(psi) + x_z*math.sin(psi) - y_j*math.cos(psi) + y_z*math.cos(psi)
    a_21 =  0
    a_22 =  0
    a_23 =  x_j*math.cos(psi) - x_z*math.cos(psi) - y_j*math.sin(psi) + y_z*math.sin(psi)
    a_31 =  0
    a_32 =  0
    a_33 =  0
    
    b_11 =  math.cos(psi)
    b_12 =  -math.sin(psi)
    b_13 =  0
    b_21 =  math.sin(psi)
    b_22 =  math.cos(psi)
    b_23 =  0
    b_31 =  0
    b_32 =  0
    b_33 =  -(math.sin(alpha)*math.cos(psi) - math.sin(psi)*math.cos(alpha))/math.sqrt(1 - (math.sin(alpha)*math.sin(psi) + math.cos(alpha)*math.cos(psi))**2)

    A = np.array([ [a_11, a_12, a_13],
            [a_21, a_22, a_23],
            [a_31, a_32, a_33]])
    B = np.array([ [b_11, b_12, b_13],
            [b_21, b_22, b_23],
            [b_31, b_32, b_33]])

    information_matrix = np.array([[400,   0,   0],
                                   [  0, 400,   0],
                                   [  0,   0, 100]])
    i = edge_source
    j = edge_target
    h_ii = A.transpose() @ information_matrix @ A
    h_ij = A.transpose() @ information_matrix @ B
    h_ji = B.transpose() @ information_matrix @ A
    h_jj = B.transpose() @ information_matrix @ B

    start_index_i = int(i) * 3
    start_index_j = int(j) * 3

    H[start_index_i:start_index_i+3, start_index_i:start_index_i+3] += h_ii
    H[start_index_i:start_index_i+3, start_index_j:start_index_j+3] += h_ij
    H[start_index_j:start_index_j+3, start_index_i:start_index_i+3] += h_ji
    H[start_index_j:start_index_j+3, start_index_j:start_index_j+3] += h_jj

    e_ij_x = (x_j - x_i) - (x_z - x_i)
    e_ij_y = (y_j - y_i) - (y_z - x_i)
    e_ij_theta = alpha - psi

    e_ij = np.array([[e_ij_x],
                     [e_ij_y],
                     [e_ij_theta]])

    b_i = A.transpose() @ information_matrix @ e_ij
    b_j = B.transpose() @ information_matrix @ e_ij
    b[int(edge_source)*3]   += b_i[0]
    b[int(edge_source)*3 + 1] += b_i[1]
    b[int(edge_source)*3 + 2] += b_i[2]

    b[int(edge_target)*3 ]   += b_j[0]
    b[int(edge_target)*3 +1] += b_j[1]
    b[int(edge_target)*3 +2] += b_j[2]
factor = cholesky


