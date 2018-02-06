#!/usr/bin/python2.7

import sys
import rospy
import logging
import pickle
from DataGenerator import DataGenerator, DataSet
import multiprocessing as mp

def start_robot(robot, result):
    ms, tt = robot.start_listener()
    result[robot.id] = robot.stn

if __name__ == "__main__":

    if len(sys.argv) < 2:
        exit(1)

    data_dir = '../../data/'
    file_name = 'dataset' + sys.argv[1] + '.pickle'
    
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

    pickle.dump(dataset, open(data_dir + file_name, 'wb'))