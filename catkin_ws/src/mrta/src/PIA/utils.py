import numpy as np
from mrta.msg import Task as Task_Msg
from Task import Task

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
    return task

