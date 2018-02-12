import pickle
import os
import sys

from DataGenerator import DataGenerator, DataSet

def compute_stats(all_schedules):
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
    return avg_makespan, avg_time_travelled

def add_pia_results(report):
    data_dir = '../data/'
    file_name = 'dataset' + sys.argv[1] + '_pia_result.pickle'

    dataset = pickle.load(open(data_dir + file_name))

    all_schedules = dataset.schedules

    avg_makespan, avg_time_travelled = compute_stats(all_schedules)

    report += "-------------\n"
    report += "Auction Based\n"
    report += "-------------\n"
    report += "alpha: " + str(dataset.bid_alpha) + "\n"
    report += "beta: " + str(dataset.beta) + "\n"
    report += "Makespan: " + str(avg_makespan) + "\n"
    report += "Total Time Travelled: " + str(avg_time_travelled) + "\n"
    report += "\n"

    i = 1
    for stn in dataset.schedules[0]:
        report += "Robot" + str(i) + "\n"
        report += str(stn)
        report += "\n"
        i += 1 

    return report

def add_dcop_results(report):
    data_dir = '../data/'
    file_name = 'dataset' + sys.argv[1] + '_dcop_result.pickle'

    dataset = pickle.load(open(data_dir + file_name))
    all_schedules = dataset.schedules

    avg_makespan, avg_time_travelled = compute_stats(all_schedules)

    report += "----\n"
    report += "DCOP\n"
    report += "----\n"
    report += "Makespan: " + str(avg_makespan) + "\n"
    report += "Total Time Travelled: " + str(avg_time_travelled) + "\n"
    report += "\n"

    i = 1
    for stn in dataset.schedules[0]:
        report += "Robot" + str(i) + "\n"
        report += str(stn)
        report += "\n" 
        i += 1

    return report


if __name__ == "__main__":
    
    if len(sys.argv) < 2:
        exit(1)

    report = "2 Robots 8 Tasks\n\n"
    report = add_pia_results(report)   
    report = add_dcop_results(report) 
    print report

