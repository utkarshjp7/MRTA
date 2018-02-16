import os
import sys
import pickle
import argparse
from copy import deepcopy

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + "/PIA/"))
sys.path.append(os.path.abspath(cur_dir + "/DCOP"))

from PIA2 import PIA
from DcopAllocator import DcopAllocator
from Robot import Robot
from DataGenerator import DataGenerator, DataSet
import random

def log_results(all_schedules, beta, alpha, task_count, robot_count, isDcop):
    avg_makespan = 0
    avg_time_travelled = 0

    total_travel_time = 0
    total_make_span = 0

    total_tasks_scheduled = 0

    for schedules in all_schedules:
        all_tasks = set()
        makespan = float('-inf')        
        for stn in schedules:
            ms = stn.get_makespan()
            tt = stn.total_travel_time
            if ms > makespan:
                makespan = ms
            total_travel_time += tt
            all_tasks = all_tasks.union(stn.get_all_tasks())

        total_tasks_scheduled += len(all_tasks)
        total_make_span += makespan
        
    avg_makespan = total_make_span / float(len(all_schedules))
    avg_time_travelled = total_travel_time / float(len(all_schedules))

    if avg_makespan == float("inf"):
        print("ERROR: Makespan can not be infinity.")
        sys.exit(0)

    if isDcop:
        print("DCOP")
    else:
        print("PIA")

    print("Number of tasks scheduled: {0}".format(total_tasks_scheduled))
    print("Average makespan: {0}".format(avg_makespan))
    print("Average time travelled: {0}".format(avg_time_travelled))

    print("-------------------------------------------------------------\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MRTA Algorithms")

    parser.add_argument('--pgraphs',
        help='Number of precedence graphs',
        dest='num_of_pgraphs',
        type=int,
        default=5,
        action='store')

    parser.add_argument('--x',
        help='X Dimention of Map',
        dest='map_x',
        type=int,
        default=100,
        action='store')

    parser.add_argument('--y',
        help='Y Dimention of Map',
        dest='map_y',
        type=int,
        default=100,
        action='store')

    robot_count_arr = [2, 4, 8, 16]
    robot_count_arr = [2, 4]
    task_count_arr = [1, 5, 10, 20, 30]
    task_count_arr = [1, 5, 10]
    alpha_arr = [0.1, 0.25, 0.5, 0.75, 1.0] 
    beta_arr = [0.1, 0.25, 0.5, 0.75, 1.0]

    args = parser.parse_args()
    map_x = args.map_x
    map_y = args.map_y
    num_of_pgraphs = args.num_of_pgraphs

    dg = DataGenerator(map_x, map_y)
    for robot_count in robot_count_arr:
        ori_robots =  dg.generate_robots(robot_count, 1)        
        for task_count in task_count_arr:
            max_possible_edges = (task_count * (task_count - 1))/2
            max_num_of_edges = min(3 * task_count, max_possible_edges)            
            for alpha in alpha_arr:
                for beta in beta_arr:
                    print("\n-------------------------------------------------------------")
                    print("Robot count: {0}".format(robot_count))
                    print("Task count: {0}".format(task_count))  
                    print("Precedence graph count: {0}".format(num_of_pgraphs))                  
                    print("Total Tasks: {0}".format(num_of_pgraphs * task_count))
                    print("Alpha: {0}".format(alpha))
                    print("Beta: {0}".format(beta))

                    all_pia_schedules = []
                    all_dcop_schedules = []

                    p_graphs = dg.generate_dataset(task_count, num_of_pgraphs, max_num_of_edges, beta)
                    for p_graph in p_graphs:
                        
                        robots1 = deepcopy(ori_robots)
                        robots2 = deepcopy(ori_robots)
                        for robot in robots1:
                            robot.set_alpha(alpha)
                        for robot in robots2:
                            robot.set_alpha(alpha)
                        
                    #    pia = PIA(deepcopy(p_graph), robots1)
                        dcop = DcopAllocator(deepcopy(p_graph))
                        
                    #    pia_schedules = pia.allocate_tasks()
                        dcop_schedules = dcop.allocate(deepcopy(robots2))

                    #    all_pia_schedules.append(pia_schedules)
                        all_dcop_schedules.append(dcop_schedules)

                    #log_results(all_pia_schedules, beta, alpha, task_count, robot_count, False)
                    log_results(all_dcop_schedules, beta, alpha, task_count, robot_count, True)
                                   