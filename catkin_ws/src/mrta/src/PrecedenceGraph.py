#!/usr/bin/python2.7

from copy import deepcopy
import numpy as np

#A vertex for the precedence graph
#
#
class Vertex:
    def __init__(self, task):
        self.task = task
        self.children = set()
        self.priority = None

        #veriables for tarjan's algorithm for finding cycles in graph
        self.tarjan_index = -1
        self.tarjan_low_link = None

        self.longest_paths = None

    def __eq__(self, other):
        return self.task == other.task 

    def __str__(self):
        s = "Task: " + str(self.task) + ", "
        s += "Children: [ "
        for c in self.children:            
            s += str(c.task) + ","
        s += " ], "
        s += "Priority: " + str(self.priority)

        return s

    def __hash__(self):
        return hash(self.task)        

    def connect(self, vertex):               
        self.children.add(vertex)
    
    def disconnect(self, vertex):
        self.children.discard(vertex)        
    
    def reset_tarjan_params(self):
        self.tarjan_index = -1
        self.tarjan_low_link = -1

# A directed acyclic graph structure
#
#

class PrecedenceGraph:
    
    def __init__(self, task_list):          
        self._vertices = set([ Vertex(task) for task in task_list ])        
        self._alpha = 0.5

        #variables for tarjan's algorithm for finding cycles in graph
        self._tarjan_scc = []
        self.scheduled_tasks = set()
        self.first_layer = set()
        self.second_layer = set()
        self.hidden_layer = set()

    def __iter__(self):        
        return iter(self._vertices)

    def size(self):
        return len(self._vertices)

    def add_vertex(self, task):                        
        self._vertices.add(Vertex(task))

    def remove_vertex(self, task):
        vertex = self._get_vertex(task)
        
        if vertex:
            self._vertices.remove(vertex)
        
        for v in self._vertices:
            v.children.discard(vertex)

    def add_edge(self, from_task, to_task):
        if not self._is_valid(from_task, to_task):
            return False

        from_vertex = self._get_vertex(from_task)
        to_vertex = self._get_vertex(to_task)

        from_vertex.connect(to_vertex)

        if self._is_cyclic():
            self.remove_edge(from_task, to_task)
            return False

        return True

    def remove_edge(self, from_task, to_task):
        if not self._is_valid(from_task, to_task):
            return

        from_vertex = self._get_vertex(from_task)
        to_vertex = self._get_vertex(to_task)

        from_vertex.disconnect(to_vertex)                

    def are_connected(self, from_task, to_task):
        if not self._is_valid(from_task, to_task):
            return False
        
        from_vertex = self._get_vertex(from_task)
        to_vertex = self._get_vertex(to_task)

        if to_vertex not in from_vertex.children:
            return False
        
        return True
        
    def calc_all_priorities(self):
        roots = self.first_layer

        for v in roots:
            self._calc_chain_priority(v)

    def update(self, scheduled_task):
        pc = {}
        v = self._get_vertex(scheduled_task)
        self.first_layer.remove(scheduled_task)
        self.scheduled_tasks.add(scheduled_task)
        
        for c in self.second_layer.intersection(v.children):
            parents = self._get_parents(c)
            if parents.issubset(self.scheduled_tasks):
                self.second_layer.remove(c)
                self.first_layer.add(c)

                pc[c] = max([ p.finish_time for p in parents ])

                for h in self.hidden_layer.intersection(c.children):
                    parents = self._get_parents(h)
                    if parents.issubset(self.scheduled_tasks.union(self.first_layer)):
                        self.hidden_layer.remove(h)
                        self.second_layer.add(h)

        return pc

    def update_tasks(self, tasks):
        for task in tasks:
            v = self._get_vertex(task)
            v.task = task

    def build_graph(self):
        self._update_first_layer()
        self._update_second_layer()
        self._update_hidden_layer()

    def _update_first_layer(self):
        self.first_layer = set()

        all_children = set()
        for v in self._vertices:
            all_children = all_children.union(v.children)

        for v in self._vertices:
            if v not in all_children:
                self.first_layer.add(v)

    def _update_second_layer(self):
        new_graph = deepcopy(self)
        
        for v in self.first_layer:
            new_graph.remove_vertex(v.task)
            
        new_graph._update_first_layer()

        self.second_layer = new_graph.first_layer

    def _update_hidden_layer(self):
        first_two_layers = self.first_layer.union(self.second_layer)
        new_graph = deepcopy(self) 

        for v in first_two_layers:
            new_graph.remove_vertex(v.task)
        
        self.hidden_layer = new_graph._vertices   

    def _get_parents(self, child):
        parents = set()
        for v in self._vertices:
            if child in v.children:
                parents.add(v.task)
        return parents

    def _is_valid(self, *tasks):
        #check for duplicates
        for i in range(0, len(tasks)):
            for j in range(i + 1, len(tasks)):
                if tasks[i] == tasks[j]:
                    return False

        #check if tasks exist in the graph
        for task in tasks:
            vertex = self._get_vertex(task)
            if vertex is None:
                return False
        
        return True            

    def _get_vertex(self, task):
        for vertex in self._vertices:
            if vertex.task == task:
                return vertex

    # Tarjan's strongly connected components algorithm
    def _is_cyclic(self):
        S = []
        index = 0
        self._tarjan_scc = []  

        for vertex in self._vertices:
            if vertex.tarjan_index == -1:
                self._strongconnect(vertex, S, index)

        for vertex in self._vertices:
            vertex.reset_tarjan_params()

        for c in self._tarjan_scc:
            if len(c) > 1:
                return True
                
        return False

    def _strongconnect(self, cur_vertex, S, index):

        cur_vertex.tarjan_index = index
        cur_vertex.tarjan_low_link = index
        index += 1
        S.append(cur_vertex)

        for child_vertex in cur_vertex.children:
            if child_vertex.tarjan_index == -1:
                self._strongconnect(child_vertex, S, index)
                cur_vertex.tarjan_low_link = min(cur_vertex.tarjan_low_link, child_vertex.tarjan_low_link)
            elif child_vertex in S:
                cur_vertex.tarjan_low_link = min(cur_vertex.tarjan_low_link, child_vertex.tarjan_index)

        if cur_vertex.tarjan_low_link == cur_vertex.tarjan_index:
            scc = []            
            while True:
                child_vertex = S.pop()
                scc.append(child_vertex.task)
                if child_vertex == cur_vertex:
                    break

            self._tarjan_scc.append(scc)
    
    def _calc_chain_priority(self, vertex):
        duration = vertex.task.duration
        max_c_lt = 0
        max_c_ut = 0
        for c in vertex.children:
            vectorAB = np.subtract(vertex.task.location, c.task.location)
            dis = np.sqrt(np.dot(vectorAB, vectorAB))
            if not c.priority:
                self._calc_chain_priority(c)

            c_lt, c_ut = c.longest_paths

            max_c_lt = c_lt if c_lt > max_c_lt else max_c_lt            
            max_c_ut = (c_ut + dis) if (c_ut + dis) > max_c_ut else max_c_ut 

        lt = duration + max_c_lt
        ut = duration + max_c_ut

        vertex.longest_paths = (lt, ut)                        
        vertex.priority = (1 - self._alpha) * lt + self._alpha * ut