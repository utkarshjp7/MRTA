#!/usr/bin/python2.7

import networkx as nx
import numpy as np
import utils
from Task import Task
from copy import deepcopy
import math
from bitarray import bitarray

class STN(object):

    def __init__(self, robot_pos, robot_speed):
        self.total_travel_time = 0 
        self.task_count = 0

        self._digraph = nx.DiGraph()
        self._robot_speed = robot_speed    
        
        zero_task = Task(0, 0, 0, -1, robot_pos[0], robot_pos[1], 0) #dummy task
        self._digraph.add_node("zero", task=zero_task, index=-1)
        
    def __str__(self):
        s = ""
        for i in range(self.task_count):
            node = self._get_node(i, _type="start")
            s += str(node[1]['task'])
            s += ", index: "
            s += str(node[1]['index'])
            s += "\n"
        return s

    def insert_task(self, task, index=None, time=None):
        if time is not None:
            index = self.task_count
            for i in range(self.task_count):
                t = self._get_task(index=i)
                if time <= t.finish_time:
                    index = i
                    break 

        if index is None:
            index = self.task_count
        
        if index < 0 or index > self.task_count:
            return

        self._increase_indexes(index)

        start_node_id = str(task.id) + 's'
        end_node_id = str(task.id) + 'e'
        zero_node_id = self._get_node(-1)[0]

        self._digraph.add_node(start_node_id, type='start', task=task, index=index)
        self._digraph.add_node(end_node_id, type='end', index=index)

        self._add_temporal_constraint(zero_node_id, start_node_id, task.lst, task.est)
        self._add_temporal_constraint(zero_node_id, end_node_id, task.lft, task.eft)
        self._add_temporal_constraint(start_node_id, end_node_id, task.duration, task.duration)                

        additional_travel_time = 0

        previous_node = self._get_node(index-1, _type='end')
        previous_node_task = self._get_task(index-1)
        travel_time = self._compute_travel_time(previous_node_task.location, task.location)   
        if index != 0:
            self._add_temporal_constraint(previous_node[0], start_node_id, travel_time, travel_time)            
        additional_travel_time += travel_time

        next_node = self._get_node(index+1, _type='start')          
        if next_node is not None:          
            next_node_task = self._get_task(index+1)
            travel_time = self._compute_travel_time(next_node_task.location, task.location)
            self._add_temporal_constraint(end_node_id, next_node[0], travel_time, travel_time)
            additional_travel_time += travel_time
            additional_travel_time -= self._compute_travel_time(previous_node_task.location, next_node_task.location)  
        
        self.task_count += 1
        self.total_travel_time += additional_travel_time

    def is_consistent(self):
        """
        Returns whether a Simple Temporal Network is consistent.
        """        
        for i in range(self.task_count):
            if self._get_task(i).start_time == float('inf'):
                return False

        return True

    def solve_stn(self, preconditions):
        self._get_task(-1).start_time = 0
        self._get_task(-1).finish_time = 0
        
        for i in range(self.task_count):
            cur_task = self._get_task(i)
            pre_task = self._get_task(i-1)        
          
            travel_time = int(math.ceil(self._compute_travel_time(cur_task.location, pre_task.location)))
            if cur_task in preconditions:
                start_time = max(preconditions[cur_task], cur_task.est, pre_task.finish_time + travel_time + 1)
            else:
                start_time = max(cur_task.est, pre_task.finish_time + travel_time + 1)

            cur_task.start_time = start_time if start_time <= cur_task.lst else float('inf')  
            cur_task.finish_time = cur_task.start_time + cur_task.duration - 1

    def get_makespan(self):
        if self.task_count <= 0:
            return 0

        return self._get_task(self.task_count - 1).finish_time    

    def update_task_constraints(self, task_id):
        task = self._get_task(task_id= task_id)

        zero_node_id = self._get_node(index=-1)[0]
        start_node_id = str(task.id) + 's'
        end_node_id = str(task.id) + 'e'

        self._add_temporal_constraint(zero_node_id, start_node_id, task.lst, task.est)
        self._add_temporal_constraint(zero_node_id, end_node_id, task.lft, task.eft)

    def get_all_tasks(self):
        all_tasks = []
        for i in range(self.task_count):
            all_tasks.append(self._get_task(i))
        
        return all_tasks

    def to_bit_arr(self):
        bit_arr = bitarray()
        for i in range(self.task_count):
            prev_task = self._get_task(i-1)
            task = self._get_task(i)
            tt = int(math.ceil(self._compute_travel_time(prev_task.location, task.location)))
            bit_arr.extend([1] * tt)
            arrival_time = prev_task.finish_time + tt            
            idle_time = task.start_time - arrival_time - 1
            bit_arr.extend([0] * idle_time)
            bit_arr.extend([1] * task.duration)
        return bit_arr

    def _get_node(self, index=None, task_id=None, _type=None):
        result = None
                
        if index == -1 or task_id is not None:
            _type = None

        if index is not None:
            result = filter(lambda (n,d): d['index'] == index, self._digraph.nodes(data=True))
        elif task_id is not None:
            result = filter(lambda (n,d): d['task'].id == task_id if 'task' in d else False, self._digraph.nodes(data=True))        

        if _type is not None:
            result = filter(lambda (n,d): d['type'] == _type, result)

        if len(result) > 0:
            return result[0]
        else:
            return None

    def _increase_indexes(self, from_index):
        for i in reversed(range(from_index, self.task_count)):
            start_node = self._get_node(i, _type="start")
            end_node = self._get_node(i, _type="end")
            start_node[1]['index'] += 1
            end_node[1]['index'] += 1

    def _get_task(self, index=None, task_id=None):
        node = None
        
        if index is not None:
            if index == -1:
                node = self._get_node(index)
            else:
                node = self._get_node(index, _type='start')
        elif task_id is not None:
            node = self._get_node(task_id=task_id)

        if node and 'task' in node[1]:
            return node[1]['task']  

    def _compute_travel_time(self, pos1, pos2):
        dis = utils.compute_distance(pos1, pos2)
        return dis / self._robot_speed

    def _add_temporal_constraint(self, start, end, upper_bound, lower_bound):
        """
        Adds a temporal constraint to the Networkx object representing the 
        distance graph.
        """
        #print start + " " + end + " = [ " + str(lower_bound) + ", " + str(upper_bound) + " ]."  
        self._digraph.add_edge(start, end, weight= upper_bound)
        self._digraph.add_edge(end, start, weight= -lower_bound)            

    def _remove_temporal_constraint(self, tc):
        """
        Removes a temporal constraint from the Networkx object representing the 
        distance graph.
        """
        self._digraph.remove_edge(tc.start, tc.end)
        self._digraph.remove_edge(tc.end, tc.start)  