#!/usr/bin/python2.7

from copy import deepcopy
import numpy as np

#A node for the precedence graph
#
#
class Node:
    def __init__(self, task):
        self.task = task
        self.children = set()
        self.priority = None

        #variables for tarjan's algorithm for finding cycles in graph
        self.tarjan_index = -1
        self.tarjan_low_link = None

        self.longest_paths = None

    def __eq__(self, other):
        return self.task == other.task 

    def __str__(self):
        s = "Task: " + str(self.task) + ", "
        s += "Children: [ "
        for c in self.children:            
            s += str(c.task.id) + ","
        s += " ], "
        s += "Priority: " + str(self.priority)

        return s

    def __hash__(self):
        return hash(self.task)        

    def connect(self, node):               
        self.children.add(node)
    
    def disconnect(self, node):
        self.children.discard(node)        
    
    def reset_tarjan_params(self):
        self.tarjan_index = -1
        self.tarjan_low_link = -1

# A directed acyclic graph structure
#
#

class PrecedenceGraph:
    
    def __init__(self, task_list):          
        self._nodes = set([ Node(task) for task in task_list ])        

        #variables for tarjan's algorithm for finding cycles in graph
        self._tarjan_scc = []
        self.scheduled_nodes = set()
        self.first_layer = set()
        self.second_layer = set()
        self.hidden_layer = set()

    def __iter__(self):        
        return iter(self._nodes)

    def size(self):
        return len(self._nodes)

    def add_node(self, task):                        
        self._nodes.add(Node(task))

    def remove_node(self, task):
        node = self._get_node(task)
        
        if node:
            self._nodes.remove(node)
        
        for node in self._nodes:
            node.children.discard(node)

    def add_edge(self, from_task, to_task):
        if not self._is_valid(from_task, to_task):
            return False

        from_node = self._get_node(from_task)
        to_node = self._get_node(to_task)

        from_node.connect(to_node)

        if self._is_cyclic():
            self.remove_edge(from_task, to_task)
            return False

        return True

    def remove_edge(self, from_task, to_task):
        if not self._is_valid(from_task, to_task):
            return

        from_node = self._get_node(from_task)
        to_node = self._get_node(to_task)

        from_node.disconnect(to_node)                

    def are_connected(self, from_task, to_task):
        if not self._is_valid(from_task, to_task):
            return False
        
        from_node = self._get_node(from_task)
        to_node = self._get_node(to_task)

        if to_node not in from_node.children:
            return False
        
        return True
        
    def calc_all_priorities(self, beta):
        roots = self.first_layer

        for v in roots:
            self._calc_chain_priority(v, beta)
    
    def get_all_tasks(self):
        tasks = []
        for node in self._nodes:
            tasks.append(node.task)
        
        return tasks

    def update(self, scheduled_task):
        pc = {}
        scheduled_task_node = self._get_node(scheduled_task)
        self.first_layer.remove(scheduled_task_node)
        self.scheduled_nodes.add(scheduled_task_node)
        
        for c in self.second_layer.intersection(scheduled_task_node.children):
            parents = self._get_parents(c)
            if parents.issubset(self.scheduled_nodes):
                self.second_layer.remove(c)
                self.first_layer.add(c)

                pc[c.task] = max([ p.task.finish_time + 1 for p in parents ])

                for h in self.hidden_layer.intersection(c.children):
                    parents = self._get_parents(h)
                    if parents.issubset(self.scheduled_nodes.union(self.first_layer)):
                        self.hidden_layer.remove(h)
                        self.second_layer.add(h)

        return pc

    def update_tasks(self, tasks):
        for task in tasks:
            node = self._get_node(task)
            node.task = task

    def build_graph(self):
        self._update_first_layer()
        self._update_second_layer()
        self._update_hidden_layer()

    def _update_first_layer(self):
        self.first_layer = set()

        all_children = set()
        for node in self._nodes:
            all_children = all_children.union(node.children)

        for node in self._nodes:
            if node not in all_children:
                self.first_layer.add(node)

    def _update_second_layer(self):
        new_graph = deepcopy(self)
        
        for node in self.first_layer:
            new_graph.remove_node(node.task)
    
        new_graph._update_first_layer()

        for node in self._nodes:
            if node in new_graph.first_layer:
                self.second_layer.add(node)

    def _update_hidden_layer(self):
        first_two_layers = self.first_layer.union(self.second_layer)
        new_graph = deepcopy(self)

        for node in first_two_layers:
            new_graph.remove_node(node.task)
        
        for node in self._nodes:
            if node in new_graph._nodes:
                self.hidden_layer.add(node)   

    def _get_parents(self, child):
        parents = set()
        for node in self._nodes:
            if child in node.children:
                parents.add(node)
        return parents

    def _is_valid(self, *tasks):
        #check for duplicates
        for i in range(0, len(tasks)):
            for j in range(i + 1, len(tasks)):
                if tasks[i] == tasks[j]:
                    return False

        #check if tasks exist in the graph
        for task in tasks:
            node = self._get_node(task)
            if node is None:
                return False
        
        return True            

    def _get_node(self, task):
        for node in self._nodes:
            if node.task == task:
                return node

    # Tarjan's strongly connected components algorithm
    def _is_cyclic(self):
        S = []
        index = 0
        self._tarjan_scc = []  

        for node in self._nodes:
            if node.tarjan_index == -1:
                self._strongconnect(node, S, index)

        for node in self._nodes:
            node.reset_tarjan_params()

        for c in self._tarjan_scc:
            if len(c) > 1:
                return True
                
        return False

    def _strongconnect(self, cur_node, S, index):

        cur_node.tarjan_index = index
        cur_node.tarjan_low_link = index
        index += 1
        S.append(cur_node)

        for child_node in cur_node.children:
            if child_node.tarjan_index == -1:
                self._strongconnect(child_node, S, index)
                cur_node.tarjan_low_link = min(cur_node.tarjan_low_link, child_node.tarjan_low_link)
            elif child_node in S:
                cur_node.tarjan_low_link = min(cur_node.tarjan_low_link, child_node.tarjan_index)

        if cur_node.tarjan_low_link == cur_node.tarjan_index:
            scc = []            
            while True:
                child_node = S.pop()
                scc.append(child_node.task)
                if child_node == cur_node:
                    break

            self._tarjan_scc.append(scc)
    
    def _calc_chain_priority(self, node, beta):
        duration = node.task.duration
        max_c_lt = 0
        max_c_ut = 0
        for c in node.children:
            vectorAB = np.subtract(node.task.location, c.task.location)
            dis = np.sqrt(np.dot(vectorAB, vectorAB))
            if not c.priority:
                self._calc_chain_priority(c, beta)

            c_lt, c_ut = c.longest_paths

            max_c_lt = c_lt if c_lt > max_c_lt else max_c_lt            
            max_c_ut = (c_ut + dis) if (c_ut + dis) > max_c_ut else max_c_ut 

        lt = duration + max_c_lt
        ut = duration + max_c_ut

        node.longest_paths = (lt, ut)                        
        node.priority = (1 - beta) * lt + beta * ut
