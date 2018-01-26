from numpy import random 
from Task import Task
from Robot import Robot
from PrecedenceGraph import PrecedenceGraph, Node

class DataGenerator:

    def __init__(self, map_size_x, map_size_y):
        self._map_size = (map_size_x, map_size_y)

    def generate_tasks(self, num_of_tasks):
        tasks = []
        duration = random.randint(20, 40)

        for i in range(num_of_tasks):
            esf = random.randint(25, 400)
            lft = esf + random.randint(100, 1200)
            pos_x = random.randint(0, self._map_size[0])
            pos_y = random.randint(0, self._map_size[1])
            tasks.append(Task(esf, lft, duration, i, pos_x, pos_y))
        
        return tasks

    def generate_precedence_graph(self, tasks, max_num_of_edges):
        p_graph = PrecedenceGraph(tasks)

        i = 0
        while i < max_num_of_edges:
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

    def generate_robot(self, _id):          
        x = random.randint(0, self._map_size[0])
        y = random.randint(0, self._map_size[1])
        robot = Robot(_id, x, y)
        return robot
