#!/usr/bin/python2.7

from DataGenerator import DataGenerator
from PIA import PIA
import pickle

if __name__ == "__main__":

    dg = DataGenerator(10, 10)
    tasks = dg.generate_tasks(10)
    p_graph = dg.generate_precedence_graph(tasks, 20)
    #robots = dg.generate_robots(2)

    pickle.dump(p_graph, open('/tmp/p_graph.pickle', 'wb'))
    #allocater = PIA(p_graph, robots)
    #allocater.start_allocation()
