from bitarray import bitarray
import utils
import math
from Task import Task

class BitSchedule():

    def __init__(self, robot_pos, robot_speed, logger, stn=None):                                
        self._robot_speed = robot_speed
        self._logger = logger
        zero_task = Task(0, 0, 0, -1, robot_pos[0], robot_pos[1], 0) #dummy task representing robot's initial position
        self._tasks = [ zero_task ]

        self._bit_arr = bitarray()                
        if stn is not None:
            self._bit_arr = stn.to_bit_arr()
            self._tasks.extend(stn.get_all_tasks())

    def __str__(self):
        return str(self._bit_arr)

    def __len__(self):
        return len(self._bit_arr)

    def __getitem__(self, key):
        return self._bit_arr[key]

    @property
    def task_count(self):
        return len(self._tasks) - 1

    def extend(self, arr):
        self._bit_arr.extend(arr)

    def prepare_for_coalition(self, task):
        #print(str(self._bit_arr))
        for i in range(self.task_count+1):
            prev_task = self._tasks[i]
            next_task = None

            if i < self.task_count:
                next_task = self._tasks[i + 1]
            
            limit = float("inf")
            if next_task is not None:
                #remove old travel time
                #print("clearing schedule between finish time of task {0} and start time of task {1}".format(prev_task.id, next_task.id))                
                l = prev_task.finish_time
                r = next_task.start_time - 1
                self._modify_bit_arr(l, r, 0)
                limit = next_task.start_time              
                #print(str(self._bit_arr))

            #add travel time from prev task to new task        
            tt = utils.compute_travel_time(prev_task.location, task.location, self._robot_speed)
            l = prev_task.finish_time
            r = min(l + tt, limit)       
            #print("Addind time travel from {0} to {1}".format(prev_task.id, task.id))     
            self._modify_bit_arr(l, r, 1)
            #print(str(self._bit_arr))

            if next_task is not None:
                #add travel time from new task to next task
                tt = utils.compute_travel_time(task.location, next_task.location, self._robot_speed)
                l = next_task.start_time - tt - 1
                r = next_task.start_time - 1
                #print("Adding time travel from {0} to {1}".format(task.id, next_task.id))
                self._modify_bit_arr(l, r, 1)
                #print(str(self._bit_arr))


    def _modify_bit_arr(self, from_idx, to_idx, bit):
        from_idx = int(math.ceil(from_idx))
        to_idx = int(math.ceil(to_idx)) 
        size = to_idx - from_idx
        self._bit_arr[from_idx : to_idx] = bitarray([bit] * size)



