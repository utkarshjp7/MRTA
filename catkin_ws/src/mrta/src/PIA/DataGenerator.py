from numpy import random 
from Task import Task
from Robot import Robot
from PrecedenceGraph import PrecedenceGraph, Node
from Logger import Logger
import sys
import pickle

class DataSet:

    def __init__(self, p_graphs, robots, beta, alpha):        
        self.p_graphs = p_graphs
        self.robots = robots
        self.beta = beta
        self.bid_alpha = bid_alpha        
        self.schedules = []

class DataGenerator:

    def __init__(self, map_size_x, map_size_y):
        self._map_size = (map_size_x, map_size_y)
        self.logger = Logger()  

    def generate_tasks(self, num_of_tasks, task_locations=None):
        if task_locations is not None:
            if len(task_locations) != num_of_tasks:
                self.logger.error("generate_tasks: The number of task locations is not same as the number of tasks.")

        tasks = []
        duration = random.randint(20, 40)

        for i in range(num_of_tasks):
            task_id = i + 1
            esf = random.randint(25, 400)
            lft = esf + random.randint(100, 1200)            
            if task_locations is not None:
                pos_x = task_locations[i][0]
                pos_y = task_locations[i][1]
            else:
                pos_x, pos_y = self.generate_locations(1)[0]

            tasks.append(Task(esf, lft, duration, task_id, pos_x, pos_y))
        
        return tasks

    def generate_locations(self, num_of_locations):
        locations = []
        for i in range(num_of_locations):
            pos_x = random.randint(0, self._map_size[0])
            pos_y = random.randint(0, self._map_size[1])
            locations.append((pos_x, pos_y))
        return locations

    def generate_precedence_graph(self, tasks, max_num_of_edges, beta):
        p_graph = PrecedenceGraph(tasks, beta)
        num_of_edges = random.randint(0, max_num_of_edges)

        i = 0
        while i < num_of_edges:
            from_task = random.choice(tasks)
            to_task = random.choice(tasks)

            if from_task.lft < to_task.lft:

                if p_graph.are_connected(from_task, to_task):
                    p_graph.remove_edge(from_task, to_task)
                else:
                    if p_graph.add_edge(from_task, to_task):
                        i += 1
        
        p_graph.build_graph()
        p_graph.calc_all_priorities()
        return p_graph

    def generate_robots(self, num_of_robots, bid_alpha):
        locations = self.generate_locations(num_of_robots)          
        robots = []

        for i in range(num_of_robots):
            robot_id = i + 1
            robot = Robot(robot_id, locations[i][0], locations[i][1], bid_alpha)
            robots.append(robot)

        return robots

if __name__ == "__main__":

    num_of_tasks = 8
    num_of_robots = 2
    num_of_precedence_graphs = 10
    max_num_of_edges = 3 * num_of_tasks
    beta = 0.5
    bid_alpha = 0.5
    data_dir = '../../data/'

    if len(sys.argv) < 2:
        exit(1)

    dataset_id = sys.argv[1]    

    dg = DataGenerator(20, 20)
    task_locations = { i : (None, None) for i in range(num_of_tasks) }
    p_graphs = []
    robots = []    
            
    #generate task locations
    locations = dg.generate_locations(num_of_tasks)      

    #generate precedence graphs
    for i in range(num_of_precedence_graphs):
        tasks = dg.generate_tasks(num_of_tasks, locations)
        p_graph = dg.generate_precedence_graph(tasks, max_num_of_edges, beta)
        p_graphs.append(p_graph)

    #generate robots
    robots = dg.generate_robots(num_of_robots, bid_alpha)

    dataset = DataSet(p_graphs, robots, beta, bid_alpha)
    file_path = data_dir + 'dataset' + dataset_id + '.pickle'
    pickle.dump(dataset, open(file_path, 'wb'))  