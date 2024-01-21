import networkx as nx

pose_graph = nx.read_graphml("pose_graph.graphml")
print(pose_graph)

# a_11 =  0
# a_12 =  0
# a_13 =  -x_j*math.sin(psi) + x_z*math.sin(psi) - y_j*math.cos(psi) + y_z*math.cos(psi)
# a_21 =  0
# a_22 =  0
# a_23 =  x_j*math.cos(psi) - x_z*math.cos(psi) - y_j*math.sin(psi) + y_z*math.sin(psi)
# a_31 =  0
# a_32 =  0
# a_33 =  0
# 
# b_11 =  math.cos(psi)
# b_12 =  -math.sin(psi)
# b_13 =  0
# b_21 =  math.sin(psi)
# b_22 =  math.cos(psi)
# b_23 =  0
# b_31 =  0
# b_32 =  0
# b_33 =  -(math.sin(alpha)*math.cos(psi) - math.sin(psi)*math.cos(alpha))/math.sqrt(1 - (math.sin(alpha)*math.sin(psi) + math.cos(alpha)*math.cos(psi))**2)
