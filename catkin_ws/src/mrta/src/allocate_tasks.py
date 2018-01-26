#!/usr/bin/python2.7

from PIA import PIA
import pickle

if __name__ == "__main__":
    p_graph = pickle.load(open("/tmp/p_graph.pickle"))

    pia = PIA(p_graph)
    pia.start_allocation()