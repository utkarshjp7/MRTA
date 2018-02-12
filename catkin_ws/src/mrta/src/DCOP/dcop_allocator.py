import pickle
import math
import sys
import os
from itertools import product

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + "/.."))
sys.path.append(os.path.abspath(cur_dir + '/graph'))
sys.path.append(os.path.abspath(cur_dir + '/function'))
sys.path.append(os.path.abspath(cur_dir + '/maxsum'))
sys.path.append(os.path.abspath(cur_dir + '/system'))
sys.path.append(os.path.abspath(cur_dir + '/solver'))

from DataGenerator import DataSet
from NodeFunction import NodeFunction
from TabularFunction import TabularFunction
from Agent import Agent
from NodeVariable import NodeVariable
from NodeFunction import NodeFunction
from NodeArgument import NodeArgument
from COP_Instance import COP_Instance
from MaxSum import MaxSum

class DcopAllocator:

    def __init__(self, p_graph):
        self._p_graph = p_graph
        self._utility_table = {} # { task_id : { robot_id : (utility, stn_pos) } }

    def allocate(self, robots):        
        n = self._p_graph.size()

        while True:
            ts = self._p_graph.scheduled_nodes
            tf = self._p_graph.first_layer
            tl = self._p_graph.second_layer

            if len(ts) >= n:
                break

            c = max([ node.priority for node in tl ]) if tl else 0
            cur_tasks = set([ node.task for node in tf if node.priority > c ])   
            
            while len(cur_tasks) > 0:
                dcop = self.create_dcop(cur_tasks, robots)
                results = self.solve_dcop(dcop)
                schedued_tasks = set()                                
                for robot_id, task_id in results.iteritems():
                    robot = [robot for robot in robots if robot.id == robot_id][0]
                    schedued_task = [task for task in cur_tasks if task.id == task_id][0]
                    pos = self._utility_table[schedued_task.id][robot.id][1]
                    robot.add_task(schedued_task, pos)

                    schedued_tasks.add(schedued_task)     
            
                for task in schedued_tasks:
                    cur_tasks.remove(task)
                    self._p_graph.update(task)    

        schedules = []
        for robot in robots:
            schedules.append(robot.stn)

        return schedules           

    def solve_dcop(self, dcop):
        ms = MaxSum(dcop, "min")
        ms.setUpdateOnlyAtEnd(False) 
        ms.setIterationsNumber(2)
        ms.solve_complete()
        #report = ms.getReport()
        #print report
        return ms.get_results()

    def create_dcop(self, tasks, robots):
        variables = set()
    #    dummy_func = NodeFunction(0)
    #    dummy_func.setFunction(TabularFunction())
    #    functions = { 0: dummy_func }
        functions = {}
        agents = []
        agent = Agent(0)
    #   agent.addNodeFunction(dummy_func)

        for task in tasks:
            function = NodeFunction(task.id)
            function.setFunction(TabularFunction())
            functions[task.id] = function
            agent.addNodeFunction(function)

        for robot in robots:        
            variable = NodeVariable(robot.id)
        #    dummy_func.addNeighbour(variable)
        #    variable.addNeighbour(dummy_func)
            
        #    domain = [dummy_func.function_id]
            domain = []
            for task in tasks:
                if robot.is_capable(task):
                    min_utility, min_pos = robot.find_min_utility(task)
                    if task.id in self._utility_table:
                        self._utility_table[task.id][robot.id] = (min_utility, min_pos)
                    else:
                        self._utility_table[task.id] = {robot.id: (min_utility, min_pos)}

                    if min_utility < float("inf"):
                        function = functions[task.id]
                        domain.append(function.function_id)
                        variable.addNeighbour(function)
                        function.addNeighbour(variable)

            if len(domain) > 0:
                variable.addDomain(domain)
                variables.add(variable)
                agent.addNodeVariable(variable)

        agents.append(agent)

        for _, function in functions.iteritems():
            param_values = []
            for param in function.params:
                param_values.append([v.value for v in param.values])

            all_values = list(product(*param_values))
            for values in all_values:
                arguments = [NodeArgument(v) for v in values]
            #    utility = float("inf")
            #    if function.function_id != dummy_func.function_id:
                utility = self._calc_function_utility(function, arguments)
  
                function.getFunction().addParametersCost(arguments, utility)

        dcop = COP_Instance(variables, list(functions.values()), agents)
        return dcop

    def _calc_function_utility(self, function, arguments):
        robot_cost_arr = []
        i = 0
        while i < len(arguments):            
            assigned_function_id = arguments[i].value
            if assigned_function_id == function.function_id:
                robot_variable = function.params[i]
                robot_cost = self._utility_table[assigned_function_id][robot_variable.id_var][0]
                robot_cost_arr.append(robot_cost)
            i += 1

        function_utility = float("inf")
        if len(robot_cost_arr) > 0:
            total_cost = sum(robot_cost_arr)/float(len(robot_cost_arr))
            function_utility = math.log(total_cost)

        return function_utility


if __name__ == "__main__":

    if len(sys.argv) < 2:
        exit(1)

    data_dir = "../../data/"
    file_name = 'dataset' + sys.argv[1] + '.pickle'
    output_file_name = 'dataset' + sys.argv[1] + '_dcop_result.pickle'
    #data_dir = '/home/uko/Developer/MRTA/catkin_ws/src/mrta/data/'
    #file_name = 'dataset2.pickle'
    
    dataset = pickle.load(open(data_dir + file_name))
    p_graph = dataset.p_graphs[0]
    robots = dataset.robots
    allocator = DcopAllocator(p_graph)
    schedules = allocator.allocate(robots)

    dataset.schedules.append(schedules)

    pickle.dump(dataset, open(data_dir + output_file_name, 'wb'))
    