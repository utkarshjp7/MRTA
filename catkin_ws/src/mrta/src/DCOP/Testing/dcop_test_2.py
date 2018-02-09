import sys, os

sys.path.append(os.path.abspath('../maxsum/'))
sys.path.append(os.path.abspath('../solver/'))
sys.path.append(os.path.abspath('../system/'))
sys.path.append(os.path.abspath('../graph/'))
sys.path.append(os.path.abspath('../misc/'))
sys.path.append(os.path.abspath('../function/'))
sys.path.append(os.path.abspath('../../PIA/'))

from Agent import Agent
from NodeVariable import NodeVariable
from NodeFunction import NodeFunction
from TabularFunction import TabularFunction
from NodeArgument import NodeArgument
from COP_Instance import COP_Instance
from MaxSum import MaxSum
from PrecedenceGraph import PrecedenceGraph, Node
from DataGenerator import DataGenerator
from Task import Task

def create_dcop(graph_nodes):
    variables = []
    functions = []
    agents = []
    agent = Agent(0)  

    #TODO : Automatically build dcop using the first layer of precedence graph



if __name__ == "__main__":
    dg = DataGenerator(10, 10)
    tasks = dg.generate_tasks(10)
    p_graph = dg.generate_precedence_graph(tasks, 20, 0.5)
    p_graph.calc_all_priorities()
   
    create_dcop(p_graph.first_layer)