import networkx as nx
import matplotlib.pyplot as plt

G=nx.Graph()
G.add_nodes_from(['F_0','F_1','F_2','F_3'],type='Factor')
G.add_nodes_from(['V_0','V_1','V_2'],type='Var')
G.add_edges_from([('F_0','V_0'),('F_1','V_1'),('F_1','V_2'),('F_2','V_1'),('F_3','V_2')])
#G.add_edges_from([(1,2),(1,3),(2,3)])
pos = nx.spring_layout(G)
nx.draw(G, pos, cmap=plt.get_cmap('jet'), node_size = 500)
plt.show()
'''
G.add_nodes_from([F_0,F_1,F_2,F_3],type='Factor')
G.add_nodes_from([V_0,V_1,V_2],type='Var')
G.add_edges_from([(F_0,V_0),(F_1,V_1),(F_1,V_2),(F_2,V_1),(F_3,V_2)])
draw(G)
'''