import rospy
import utils
from mrta.msg import AuctionRequest, AuctionAck, Bid, Winner, ScheduledTasks
from STN import STN
from copy import deepcopy

class Robot:

    def __init__(self, _id, pos_x, pos_y):
        self.init_pos = (pos_x, pos_y)
        self.id = _id
        self.stn = STN(self.init_pos, 1)
        self._bid_alpha = 0.5
        
        self._auc_ack_pub = rospy.Publisher("/auction_ack", AuctionAck, queue_size=10)
        self._bid_pub = rospy.Publisher("/bid", Bid, queue_size=10)
        self._scheduled_tasks_pub = rospy.Publisher("/scheduled_tasks", ScheduledTasks, queue_size=10)
        self._auction_sub = rospy.Subscriber("/auction", AuctionRequest, self.auction_callback)
        self._winner_sub = rospy.Subscriber("/winner", Winner, self.winner_callback)

        self._t_auc = set()
        self._best_task_pos = {}
        self._winner_received = True
    
    def __str__(self):
        s = self.id
        s += "Init location:" + str(self.init_pos)
        return s
    
    def start_listener(self):
        rospy.init_node("robot" + str(self.id))
        rospy.spin()

    def auction_callback(self, msg):        
        auc_ack_msg = AuctionAck()
        auc_ack_msg.robot_id = int(self.id)
        auc_ack_msg.auc_id = int(msg.id)        
        self._auc_ack_pub.publish(auc_ack_msg)

        min_bid = float("inf")
        min_task = None

        self._t_auc = set(utils.create_tasks(msg.tasks))
        PC = msg.PC

        while len(self._t_auc) > 0:

            if self._winner_received:
                i = 0
                for task in self._t_auc:
                    bid, best_pos = self._compute_min_bid(task, PC[i])
                    self._best_task_pos[task.id] = best_pos 
                    if bid < min_bid:
                        min_bid = bid
                        min_task = task
                    i += 1

                bid_msg = Bid()
                bid_msg.auc_id = msg.id
                bid_msg.robot_id = self.id
                bid_msg.task = utils.create_task_msg(min_task)
                bid_msg.bid = min_bid

                self._bid_pub.publish(bid_msg)
                self._winner_received = False
        
        tasks = self._tighten_schedule()
        scheduled_tasks_msg = ScheduledTasks()
        scheduled_tasks_msg.robot_id = self.id
        scheduled_tasks_msg.tasks = tasks
        
        self._scheduled_tasks_pub.publish(scheduled_tasks_msg)

    def winner_callback(self, msg):
        winner_robot_id = msg.robot_id
        task = utils.create_task(msg.task)

        if winner_robot_id == self.id:            
            self.stn.insert_task(task, self._best_task_pos[task.id])
        
        self._t_auc.remove(task)
        self._winner_received = True             
    
    def _tighten_schedule(self):
        tasks = []
        for i in range(self.stn.task_count):
            task = self.stn._get_task(i)            
            f = task.finish_time

            task.update_time_window(task.est, f)
            self.stn.update_task_constraints(task.id)
            tasks.append(tasks)
        return tasks

    def _compute_min_bid(self, task, pc):
        task_count = self.stn.task_count
        min_bid = float("inf")
        min_pos = None

        for i in range(task_count + 1):   
            temp_stn = deepcopy(self.stn)

            tt_before = temp_stn.total_travel_time
            temp_stn.insert_task(task, i)
            temp_stn.solve_stn(pc)              
            tt_after = temp_stn.total_travel_time

            if temp_stn.is_consistent():
                addition_travel_time = tt_after - tt_before
                bid = self._compute_bid(temp_stn.get_makespan(), addition_travel_time)
                if bid < min_bid:
                    min_bid = bid
                    min_pos = i

        return min_bid, min_pos

    def _compute_bid(self, makespan, travel_time):
        return makespan * self._bid_alpha + travel_time * (1 - self._bid_alpha) 
