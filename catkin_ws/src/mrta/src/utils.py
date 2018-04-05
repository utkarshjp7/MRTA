import os
import sys
import numpy as np
from mrta.msg import Task as Task_Msg
from Task import Task
from bitarray import bitarray

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + "/.."))

NEG_INF = -10000000

def print_schedules(all_schedules, desc):    
    i = 0
    while i < len(all_schedules):             
        schedules = all_schedules[i]
        j = 0
        print("\n--------------------------{0}--------------------------------\n".format(desc))
        while j < len(schedules):
            if schedules[j].task_count > 0:
                print(str(schedules[j]))
                print("\n")            
            j += 1
        i += 1
    print("\n----------------------------------------------------------\n")

def calculate_stats(all_schedules):
    avg_makespan = 0
    avg_time_travelled = 0
    total_tasks_scheduled = 0

    total_travel_time = 0
    total_make_span = 0    

    for schedules in all_schedules:
        all_tasks = set()
        makespan = float('-inf')        
        for stn in schedules:
            ms = stn.get_makespan()
            tt = stn.total_travel_time
            if ms > makespan:
                makespan = ms
            total_travel_time += tt
            all_tasks = all_tasks.union(set(stn.get_all_tasks()))

        total_tasks_scheduled += len(all_tasks)
        total_make_span += makespan
        
    if len(all_schedules) != 0:
        avg_makespan = total_make_span / float(len(all_schedules))
        avg_time_travelled = total_travel_time / float(len(all_schedules))
        
        if avg_makespan == float("inf"):
            print("ERROR: Makespan can not be infinity.")
            sys.exit(0)

    return avg_makespan, avg_time_travelled, total_tasks_scheduled

def print_pgraph(pgraph):
    print(" ")

    for vertex in pgraph:
        print str(vertex)

    print "--------First Layer----------"
    tf = pgraph.first_layer

    for v in tf:
        print str(v)

    print "--------Second Layer----------"
    tl = pgraph.second_layer

    for v in tl:
        print str(v)

    print "--------Hidden Layer----------"
    th = pgraph.hidden_layer

    for v in th:
        print str(v)    
    
    print(" ")
    print("------------------------------------")
    print(" ")

def compute_distance(vecA, vecB):
    vectorAB = np.subtract(vecB, vecA)
    return round(np.sqrt(np.dot(vectorAB, vectorAB)), 4)    

def create_task_msg(task):
    task_msg = Task_Msg()
    task_msg.id = task.id
    task_msg.start_time = task.start_time
    task_msg.finish_time = task.finish_time 
    task_msg.est = task.est
    task_msg.lst = task.lst
    task_msg.eft = task.eft
    task_msg.lft = task.lft            
    task_msg.duration = task.duration
    task_msg.location = task.location
    return task_msg

def create_tasks(task_msgs):
    tasks = []
    for task_msg in task_msgs:
        tasks.append(create_task(task_msg))
    
    return tasks

def create_task(task_msg):
    task = Task(task_msg.est, task_msg.lft, task_msg.duration, task_msg.id, task_msg.location[0], task_msg.location[1])
    task.start_time = task_msg.start_time
    task.finish_time = task_msg.finish_time
    return task    

def compute_travel_time(pos1, pos2, speed):
    dis =  compute_distance(pos1, pos2)
    return dis / speed

def find_common_gap_in_bit_schedules(bitarrays, size):
        arr_len = len(bitarrays[0])
        common_time_slot = bitarray([0] * arr_len)        
        for arr in bitarrays:
            common_time_slot = common_time_slot | arr
        common_time_slot = common_time_slot.tolist()

        n = 0
        i = 0        
        for b in common_time_slot:
            if b == 0:
                n += 1
            else:
                n = 0

            if n >= size:
                return i - size + 1
                break
            i += 1

        return -1

def compute_task_cost(ms, tt, alpha):
    return ms * alpha + tt * (1 - alpha)