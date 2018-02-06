#!/usr/bin/python2.7

from PIA import PIA
import pickle
import sys
from DataGenerator import DataSet

if __name__ == "__main__":

    if len(sys.argv) < 2:
        exit(1)

    data_dir = "../../data/"
    file_name = 'dataset' + sys.argv[1] + '.pickle'
    
    dataset = pickle.load(open(data_dir + file_name))
    
    p_graph = dataset.p_graphs[0]
    robots = dataset.robots

    pia = PIA(p_graph, robots)
    pia.allocate_tasks()
