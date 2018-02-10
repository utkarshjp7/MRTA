import sys, os
import pickle
from itertools import product

sys.path.append(os.path.abspath('../maxsum/'))
sys.path.append(os.path.abspath('../solver/'))
sys.path.append(os.path.abspath('../system/'))
sys.path.append(os.path.abspath('../graph/'))
sys.path.append(os.path.abspath('../misc/'))
sys.path.append(os.path.abspath('../function/'))
sys.path.append(os.path.abspath('../../'))

from Agent import Agent
from NodeVariable import NodeVariable
from NodeFunction import NodeFunction
from TabularFunction import TabularFunction
from NodeArgument import NodeArgument
from COP_Instance import COP_Instance
from MaxSum import MaxSum
from PrecedenceGraph import PrecedenceGraph, Node
from DataGenerator import DataGenerator, DataSet
from Task import Task

def create_dcop(tasks, robots):
    variables = set()
    dummy_func = NodeFunction(0)
    dummy_func.setFunction(TabularFunction())
    functions = { 0: dummy_func }
    agents = []
    agent = Agent(0)  

    for task in tasks:
        print "Task: " + str(task.id) + " ,Type: " + str(task.type)
        function = NodeFunction(task.id)
        function.setFunction(TabularFunction())
        functions[task.id] = function
        agent.addNodeFunction(function)

    for robot in robots:        
        print "Robot: " + str(robot.id) + ", Capability: " + str(robot._capability)
        variable = NodeVariable(robot.id)
        dummy_func.addNeighbour(variable)
        variable.addNeighbour(dummy_func)
        
        domain = [0]
        for task in tasks:
            if robot.is_capable(task):
                function = functions[task.id]
                domain.append(function.function_id)
                variable.addNeighbour(function)
                function.addNeighbour(variable)

        variable.addDomain(domain)
        variables.add(variable)
        agent.addNodeVariable(variable)

    agents.append(agent)

    for _, function in functions.iteritems():
        print "Function: " + str(function.function_id) + " , No. of params: " + str(len(function.params))
        
        param_values = []
        for param in function.params:
            param_values.append(param.values)

        all_values = list(product(*param_values))
        for values in all_values:
            arguments = [NodeArgument(v) for v in values]
            cost = 0
            for arg in arguments:
                print str(arg),
            print ""
            function.getFunction().addParametersCost(arguments, cost)

    cop = COP_Instance(variables, list(functions.values()), agents)
    return cop

if __name__ == "__main__":

    if len(sys.argv) < 2:
        exit(1)

    data_dir = "../../../data/"
    file_name = 'dataset' + sys.argv[1] + '.pickle'
    dataset = pickle.load(open(data_dir + file_name))
    p_graph = dataset.p_graphs[0]
    robots = dataset.robots
    tasks = []
    for node in p_graph.first_layer:
        tasks.append(node.task)

    cop = create_dcop(tasks, robots)