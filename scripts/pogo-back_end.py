import networkx as nx
import numpy as np
import math
import sys
np.set_printoptions(threshold=np.inf)

pose_graph = nx.read_graphml("pose_graph.graphml")
print(pose_graph)
num_nodes = nx.number_of_nodes(pose_graph)
Hessian = np.zeros((num_nodes * 3, num_nodes * 3))
print(Hessian.shape)

for edge in pose_graph.edges(data=True):

    edge_source, edge_target, edge_attr = edge
    print(edge_source)
    print(edge_target)

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
    hessian_ii = A.transpose() @ information_matrix @ A
    hessian_ij = A.transpose() @ information_matrix @ B
    hessian_ji = B.transpose() @ information_matrix @ A
    hessian_jj = B.transpose() @ information_matrix @ B
    print("Hessian ii: ", hessian_ii)
    print("Hessian ij: ", hessian_ij)
    print("Hessian ji: ", hessian_ji)
    print("Hessian jj: ", hessian_jj)

    start_index_i = int(i) * 3
    start_index_j = int(j) * 3

    Hessian[start_index_i:start_index_i+3, start_index_i:start_index_i+3] = hessian_ii
    Hessian[start_index_i:start_index_i+3, start_index_j:start_index_j+3] = hessian_ij
    Hessian[start_index_j:start_index_j+3, start_index_i:start_index_i+3] = hessian_ji
    Hessian[start_index_j:start_index_j+3, start_index_j:start_index_j+3] = hessian_jj

# print(Hessian)

