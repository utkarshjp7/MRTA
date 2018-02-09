import os, sys

sys.path.append(os.path.abspath('../'))

from PrecedenceGraph import PrecedenceGraph, Node
from DataGenerator import DataGenerator
from Task import Task

def print_graph(graph):
    print(" ")

    for vertex in graph:
        print str(vertex)

    print "--------First Layer----------"
    tf = graph.first_layer

    for v in tf:
        print str(v)

    print "--------Second Layer----------"
    tl = graph.second_layer

    for v in tl:
        print str(v)

    print "--------Hidden Layer----------"
    th = graph.hidden_layer

    for v in th:
        print str(v)    
    
    print(" ")
    print("------------------------------------")
    print(" ")

x = Task(1, 1, 2, 1, 1, 1)
y = Task(1, 1, 60, 2, 2, 2)
z = Task(1, 1, 1, 3, 3, 3)
a = Task(1, 1, 10, 4, 4, 4)
b = Task(1, 1, 5, 5, 5, 5)
c = Task(1, 1, 1, 6, 6, 6)
d = Task(1, 1, 1, 7, 7, 7)

graph_1 = PrecedenceGraph([x, y, z, a, b, c, d], 0.5)
graph_1.add_edge(x, y)
graph_1.add_edge(y, z)
graph_1.add_edge(z, a)
graph_1.add_edge(a, b)
graph_1.add_edge(b, c)
graph_1.add_edge(d, b)
graph_1.calc_all_priorities()

print_graph(graph_1)

dg = DataGenerator(10, 10)
tasks = dg.generate_tasks(10)
graph_2 = dg.generate_precedence_graph(tasks, 20, 0.5)
graph_2.calc_all_priorities()
print_graph(graph_2)




