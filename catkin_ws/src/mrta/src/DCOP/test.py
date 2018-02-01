import networkx as nx
import matplotlib.pyplot as plt

G=nx.Graph()
G.add_nodes_from(['F_0','F_1','F_2','F_3'],type='Factor')
G.add_nodes_from(['V_0','V_1','V_2'],type='Var')
G.add_edges_from([('F_0','V_0'),('F_1','V_0'),('F_1','V_1'),('F_1','V_2'),('F_2','V_1'),('F_3','V_2')])

nx.draw(G,with_labels=True,node_size=1000)
plt.show()
'''
G.add_nodes_from([F_0,F_1,F_2,F_3],type='Factor')
G.add_nodes_from([V_0,V_1,V_2],type='Var')
G.add_edges_from([(F_0,V_0),(F_1,V_1),(F_1,V_2),(F_2,V_1),(F_3,V_2)])
draw(G)
'''