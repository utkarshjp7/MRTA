import os, sys

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + "/../.."))

from utils import print_pgraph
from PrecedenceGraph import PrecedenceGraph, Node
from DataGenerator import DataGenerator
from Task import Task
from Logger import Logger, LogLevel

logger = Logger(LogLevel.OFF[0])

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
graph_1.build_graph()
graph_1.calc_all_priorities()

print_pgraph(graph_1)

dg = DataGenerator(10, 10, logger)
tasks = dg.generate_tasks(10)
graph_2 = dg.generate_precedence_graph(tasks, 20, 0.5)
print_pgraph(graph_2)




