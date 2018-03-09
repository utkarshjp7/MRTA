import sys, os

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + '/../..'))
sys.path.append(os.path.abspath(cur_dir + '/..'))

from PrecedenceGraph import PrecedenceGraph, Node
from DataGenerator import DataGenerator, DataSet
from Task import Task
from DcopAllocator import DcopAllocator
from Logger import Logger, LogLevel
from copy import deepcopy
import utils
from datetime import datetime

if __name__ == "__main__":

    logger = Logger(LogLevel.OFF[0])
    dg = DataGenerator(100, 100, logger)
    tasks = dg.generate_tasks(10)
    robots = dg.generate_robots(2, 1)
    p_graph = PrecedenceGraph(tasks, 0.5)
    p_graph.build_graph()
    p_graph.calc_all_priorities()

    """
    bf = datetime.now()
    dcop2 = DcopAllocator(deepcopy(p_graph), logger)
    schedules2 = dcop2.allocate(deepcopy(robots), test=True)
    af = datetime.now()
    exec_time2 = (af - bf).total_seconds()
    
    ms, tt, st = utils.calculate_stats([schedules2])
    print "makespan: " + str(ms)
    print "time travelled: " + str(tt)
    print "tasks scheduled: " + str(st)
    print "exec time: " + str(exec_time2)
    utils.print_schedules([schedules2], 'DCOP2')
    """

    bf = datetime.now()
    dcop = DcopAllocator(deepcopy(p_graph), logger)
    schedules = dcop.allocate(deepcopy(robots))
    af = datetime.now()
    exec_time1 = (af - bf).total_seconds()

    ms, tt, st = utils.calculate_stats([schedules])
    print "makespan: " + str(ms)
    print "time travelled: " + str(tt)
    print "tasks scheduled: " + str(st)
    print "exec time: " + str(exec_time1)
    utils.print_schedules([schedules], 'DCOP')

    