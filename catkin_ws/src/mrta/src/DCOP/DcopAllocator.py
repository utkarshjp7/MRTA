import pickle
import math
import sys
import os
from itertools import product
from copy import deepcopy

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
from Logger import Logger

class DcopAllocator:

    def __init__(self, p_graph):
        self._p_graph = p_graph
        self._cost_table = {} # { task_id : { robot_id : (utility, stn_pos) } }
        self.logger = Logger()
        self._tasks_preconditions = {}

    def allocate(self, robots):        
        n = self._p_graph.size()
        if n > 2:
            x = 10

        while True:
            ts = self._p_graph.scheduled_nodes
            tf = self._p_graph.first_layer
            tl = self._p_graph.second_layer

            self.logger.debug("{0} tasks have been scheduled.".format(len(ts)))
            self.logger.debug("First layer contains {0} tasks.".format(len(tf)))
            self.logger.debug("Second layer contains {0} tasks.".format(len(tl))) 

            if len(ts) >= n:
                self.logger.debug("All tasks have been scheduled.")
                break

            c = max([ node.priority for node in tl ]) if tl else 0
            batch = set([ node.task for node in tf if node.priority > c ])
            
            cur_tasks = deepcopy(batch)
            while len(cur_tasks) > 0:
                self.logger.debug("Creating dcop for {0} tasks.".format(len(cur_tasks)))
                dcop = self.create_dcop(cur_tasks, robots)

                keepGoing = False
                for function in dcop.getNodeFunctions():
                    if len(function.params) == 0:
                        task = filter(lambda task: task.id == function.function_id, cur_tasks)[0]                        
                        cur_tasks.remove(task) #remove impossible task
                    else:
                        keepGoing = True

                if keepGoing:                    
                    self.logger.debug("Dcop created. Now solving dcop.")
                    results = self.solve_dcop(dcop)
                    self.logger.debug("Dcop solved.")

                    if len(results) == 0:
                        print("Should never happen")
                        sys.exit(0)
                                                    
                    scheduled_tasks = set()
                    for robot_id, task_id in results.iteritems():
                        self.logger.debug("Task {0} has been assigned to robot {1}".format(task_id, robot_id))
                        robot = [robot for robot in robots if robot.id == robot_id][0]
                        scheduled_task = [task for task in cur_tasks if task.id == task_id][0]
                        pos = self._cost_table[scheduled_task.id][robot.id][1]
                        self.logger.debug("Adding task {0} to robot {1}'s STN at position {2}".format(task_id, robot_id, pos))
                        robot.add_task(scheduled_task, pos, self._tasks_preconditions)
                        self.logger.debug("Task has been added.")
                        scheduled_tasks.add(scheduled_task)

                    cur_tasks = cur_tasks.difference(scheduled_tasks)  #remove scheduled tasks
            
            for task in batch:                
                self.logger.debug("Updating precedence graph for task {0}".format(task.id))
                pc = self._p_graph.update(task)  
                for k,v in pc.iteritems():
                    self._tasks_preconditions[k] = v 
                self.logger.debug("Precedence graph updated.")  

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
                    min_cost, min_pos = robot.find_min_cost(task, self._tasks_preconditions)
                    if task.id in self._cost_table:
                        self._cost_table[task.id][robot.id] = (min_cost, min_pos)
                    else:
                        self._cost_table[task.id] = {robot.id: (min_cost, min_pos)}

                    #if robot can fit this task in its schedule
                    if min_cost < float("inf"):
                        function = functions[task.id]
                        domain.append(function.function_id)
                        variable.addNeighbour(function)
                        function.addNeighbour(variable)

            #if robot can do atleast one task
            if len(domain) > 0:
                variable.addDomain(domain)
                variables.add(variable)
                agent.addNodeVariable(variable)        

        for _, function in functions.iteritems():                        
            if len(function.params) > 0:
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

        agents.append(agent)
        dcop = COP_Instance(variables, list(functions.values()), agents)
        return dcop

    def _calc_function_utility(self, function, arguments):
        robot_cost_arr = []
        i = 0
        while i < len(arguments):            
            assigned_function_id = arguments[i].value
            if assigned_function_id == function.function_id:
                robot_variable = function.params[i]
                robot_cost = self._cost_table[assigned_function_id][robot_variable.id_var][0]
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

    num_of_datasets = int(sys.argv[1])
    #num_of_datasets = 1

    for i in range(1, num_of_datasets+1):

        print "Processing dataset " + str(i)
        print("----------------------------------------------\n\n")
        data_dir = "../../data/"
        input_file_path = data_dir + 'dataset' + str(i) + '.pickle'
        output_file_path = data_dir + 'result/dcop/dataset' + str(i) + '.pickle'
        #input_file_path = '/home/uko/Developer/MRTA/catkin_ws/src/mrta/data/dataset1.pickle'
        #output_file_path = '/home/uko/Developer/MRTA/catkin_ws/src/mrta/data/result/dcop/dataset1.pickle'
        
        dataset = pickle.load(open(input_file_path))
        j = 1
        for p_graph in dataset.p_graphs:
            print("----------------------------------------------\n\n")
            print("Allocating precedence graph {0}\n".format(j))
            robots = deepcopy(dataset.robots)
            print("Task count for robot {0} is {1}".format(robots[0].id, robots[0].stn.task_count))
            allocator = DcopAllocator(p_graph)
            schedules = allocator.allocate(robots)
            dataset.schedules.append(schedules)
            j += 1
            print("\n\n----------------------------------------------")

        print("\n\n----------------------------------------------")
        print("Creating output file for dataset {0}".format(str(i)))
        pickle.dump(dataset, open(output_file_path, 'wb'))
