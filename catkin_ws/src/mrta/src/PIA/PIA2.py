import os
import sys

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + "/.."))
import utils

class PIA(object):

    def __init__(self, p_graph, robots, logger, tighten_schedule=True, use_prio=True):        
        self.logger = logger
        self.p_graph = p_graph
        self.robots = robots
        self._tighten_schedule = tighten_schedule                   
        self._use_prio = use_prio                   
        self._tasks_preconditions = {}

    def allocate_tasks(self):
        n = self.p_graph.size()
        auc_id = 1
        bids = { robot.id : (-1, None) for robot in self.robots }    

        while True :    
            ts = self.p_graph.scheduled_nodes
            tf = self.p_graph.first_layer
            tl = self.p_graph.second_layer

            self.logger.debug("{0} tasks have been scheduled.".format(len(ts)))
            self.logger.debug("First layer contains {0} tasks.".format(len(tf)))
            self.logger.debug("Second layer contains {0} tasks.".format(len(tl))) 

            if len(ts) >= n:
                self.logger.debug("All tasks have been allocated.") 
                break          

            t_auct = [node.task for node in tf]
            if self._use_prio:
                c = max([ node.priority for node in tl ]) if tl else 0
                t_auct = [ node.task for node in tf if node.priority > c ]        
            
            for v in tf:
                self.logger.debug("First layer: Task {0} with priority {1}".format(v.task.id, v.priority))

            for v in tl:
                self.logger.debug("Second layer: Task {0} with priority {1}".format(v.task.id, v.priority))                        
            
            self._init_auction(auc_id, t_auct)
            self.logger.debug("Auction {0} started with {1} tasks.".format(auc_id, len(t_auct)))
            
            i = 0
            while i < len(t_auct):
                for robot in self.robots:
                    bid, task = robot.get_min_bid()
                    self.logger.debug("Robot {0} has bid {1}".format(robot.id, bid))
                    bids[robot.id] = (bid, task)

                win_bid, win_robot_id = self._calc_winner(bids)

                if win_robot_id is None:
                    self.logger.warn("Tasks cannot be allocated")                                                         
                    for robot in self.robots:
                        robot.end_auction()
                    break                                                       

                win_task = bids[win_robot_id][1]                

                self._send_winner(auc_id, win_robot_id, win_task)

                bids = { robot.id : (-1, None) for robot in self.robots }
                i += 1

            for robot in self.robots:
                if self._tighten_schedule:
                    tasks = robot.tighten_schedule()
                else:
                    tasks = set(robot.stn.get_all_tasks())
                    
                self.logger.debug("Robot {0}: Makespan is {1}".format(robot.id, robot.stn.get_makespan()))
                self.logger.debug("\nRobot {0}: Schedule:\n {1}\n".format(robot.id, str(robot.stn)))
                self.p_graph.update_tasks(tasks)

            self.logger.debug("Auction {0} finished".format(auc_id))
            
            self._update_precedence_graph(t_auct)
            auc_id += 1

        schedules = [robot.stn for robot in self.robots]
        return schedules

    def _init_auction(self, auc_id, t_auct):
        pc = []
        for task in t_auct:        
            pc.append(self._tasks_preconditions[task] if task in self._tasks_preconditions else 0)

        #init auction
        for robot in self.robots:
            robot.init_auction(auc_id, t_auct, pc)
    

    def _calc_winner(self, bids):
        win_bid = float('inf')
        win_robot_id = None
        for robot_id, bid in bids.iteritems():
            if bid[0] < win_bid:                
                win_robot_id = robot_id
                win_bid = bid[0]
        return win_bid, win_robot_id

    def _send_winner(self, auc_id, win_robot_id, win_task):
        self.logger.info("Sending winner: Auc id: {0}, Robot id: {1}, task: {2}".format(auc_id, win_robot_id, win_task.id))
        for robot in self.robots:
            robot.notify_winner(auc_id, win_robot_id, win_task)

    
    def _update_precedence_graph(self, t_auct):
        for t in t_auct:
            self.logger.info("Updating precedence graph")
            pc = self.p_graph.update(t)
            for k,v in pc.iteritems():
                self._tasks_preconditions[k] = v   
