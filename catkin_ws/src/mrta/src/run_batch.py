import os
import sys
import pickle
import argparse
from copy import deepcopy
import psycopg2

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + "/PIA/"))
sys.path.append(os.path.abspath(cur_dir + "/DCOP"))

from PIA2 import PIA
from DcopAllocator import DcopAllocator
from Robot import Robot
from DataGenerator import DataGenerator, DataSet
import random

def execute_sql(conn, sql):
    cursor = conn.cursor()
    try:
        cursor.execute(sql)
    except psycopg2.Error as e:
        print("Database error occured while executing {0}".format(sql))
        print e.diag.message_primary
        sys.exit(0)

    conn.commit()
    cursor.close()    

def print_schedules(dcop_schedules, pia_schedules):
    
    i = 0
    while i < len(dcop_schedules):
        schedules1 = dcop_schedules[i]
        schedules2 = pia_schedules[i]

        j = 0
        while j < len(schedules1):
            if schedules1[j].task_count > 0:
                print(str(schedules1[j]))
                print("\n")            
            j += 1

        print("\n-------------------------------------------------------------\n")

        j = 0
        while j < len(schedules2):
            if schedules2[j].task_count > 0:
                print(str(schedules2[j]))
                print("\n")
            j += 1
        
        i += 1

        print("\n-------------------------------------------------------------\n")


def calculate_stats(all_schedules):
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

    return avg_makespan, avg_time_travelled, total_tasks_scheduled


def log_results(dcop_schedules, pia_schedules, beta, alpha, task_count, robot_count, num_of_pgraphs):
    dcop_ms, dcop_tt, dcop_st = calculate_stats(dcop_schedules)
    pia_ms, pia_tt, pia_st = calculate_stats(pia_schedules)

    print("DCOP: Number of tasks scheduled: {0}".format(dcop_st))
    print("DCOP: Average makespan: {0}".format(dcop_ms))
    print("DCOP: Average time travelled: {0}".format(dcop_tt))

    print("PIA: Number of tasks scheduled: {0}".format(pia_st))
    print("PIA: Average makespan: {0}".format(pia_ms))
    print("PIA: Average time travelled: {0}".format(pia_tt))
    #print_schedules(dcop_schedules, pia_schedules)
    

    connect_str = "dbname='mrta' user='uko' password='uko27415041' host='localhost'"
    conn = psycopg2.connect(connect_str)
   
    insert_record = """
                        INSERT INTO 
                            results(robots, tasks, pgraphs, alpha, beta, 
                                 pia_ms, pia_tt, pia_scheduled_tasks,
                                 dcop_ms, dcop_tt, dcop_scheduled_tasks)
                        
                        VALUES('{0}', '{1}', '{2}', '{3}', '{4}', '{5}', '{6}', '{7}', '{8}', '{9}', '{10}')
                    """.format(robot_count, task_count, num_of_pgraphs, alpha, beta, pia_ms, pia_tt, pia_st, dcop_ms, dcop_tt, dcop_st)

    execute_sql(conn, insert_record)

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
    robot_count_arr = [2]
    task_count_arr = [1, 5, 10, 20, 30]
    task_count_arr = [10]
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
                    all_pia_schedules = []
                    all_dcop_schedules = []

                    print("\n-------------------------------------------------------------")
                    print("Robot count: {0}".format(robot_count))
                    print("Task count: {0}".format(task_count))  
                    print("Precedence graph count: {0}".format(num_of_pgraphs))                  
                    print("Total Tasks: {0}".format(num_of_pgraphs * task_count))
                    print("Alpha: {0}".format(alpha))
                    print("Beta: {0}".format(beta))
                    
                    p_graphs = dg.generate_dataset(task_count, num_of_pgraphs, max_num_of_edges, beta)                                       
                    for p_graph in p_graphs:
                        
                        robots1 = deepcopy(ori_robots)
                        robots2 = deepcopy(ori_robots)
                        for robot in robots1:
                            robot.set_alpha(alpha)
                        for robot in robots2:
                            robot.set_alpha(alpha)
                                                
                        pia = PIA(deepcopy(p_graph), robots1)
                        dcop = DcopAllocator(deepcopy(p_graph))
                        
                        pia_schedules = pia.allocate_tasks()
                        dcop_schedules = dcop.allocate(deepcopy(robots2), True)

                        all_pia_schedules.append(pia_schedules)
                        all_dcop_schedules.append(dcop_schedules)

                    #log_results(all_pia_schedules, beta, alpha, task_count, robot_count, False)
                    log_results(all_dcop_schedules, all_pia_schedules, beta, alpha, task_count, robot_count, num_of_pgraphs)
                    print("-------------------------------------------------------------\n")
                                   