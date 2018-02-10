import pickle
import os
import sys

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + "/.."))
from DataGenerator import DataGenerator, DataSet

if __name__ == "__main__":
    
    if len(sys.argv) < 2:
        exit(1)

    data_dir = '../../data/'
    file_name = 'dataset' + sys.argv[1] + '.pickle'

    dataset = pickle.load(open(data_dir + file_name))

    all_schedules = dataset.schedules

    avg_makespan = 0
    avg_time_travelled = 0

    total_travel_time = 0
    total_make_span = 0

    for schedules in all_schedules:
        makespan = float('-inf')
        
        for stn in schedules:
            ms = stn.get_makespan()
            tt = stn.total_travel_time
            if ms > makespan:
                makespan = ms
            total_travel_time += tt

        total_make_span += makespan

    avg_makespan = (total_make_span / len(all_schedules))
    avg_time_travelled = total_travel_time / len(all_schedules)

    report = "2 Robots 8 Tasks\n\n"
    
    report += "alpha: " + str(dataset.bid_alpha) + "\n"
    report += "beta: " + str(dataset.beta) + "\n"
    report += "Makespan: " + str(avg_makespan) + "\n"
    report += "Total Time Travelled: " + str(avg_time_travelled) + "\n"
    report += "\n"

    for stn in dataset.schedules[0]:
        report += str(stn)
        report += "\n" 

    print report