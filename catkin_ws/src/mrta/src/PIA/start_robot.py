#!/usr/bin/python2.7

import os
import sys
import rospy
import logging
import pickle
import multiprocessing as mp

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + "/.."))
from DataGenerator import DataGenerator, DataSet

def start_robot(robot, result):
    ms, tt = robot.start_listener()
    result[robot.id] = robot.stn

if __name__ == "__main__":

    if len(sys.argv) < 2:
        exit(1)

    data_dir = '../../data/'
    file_name = 'dataset' + sys.argv[1] + '.pickle'
    output_file_name = 'dataset' + sys.argv[1] + '_pia_result.pickle'
    
    dataset = pickle.load(open(data_dir + file_name))
    result = mp.Manager().dict()

    robots = dataset.robots
    p_arr = []
    for robot in robots:
        result[robot.id] = (None, None)
        p = mp.Process(target=start_robot, args=(robot,result,))
        p_arr.append(p)
        p.start()

    for p in p_arr:
        p.join()

    schedules = list(result.values())

    dataset.schedules.append(schedules)

    pickle.dump(dataset, open(data_dir + output_file_name, 'wb'))