import os
import sys
import rospy
from mrta.msg import AuctionRequest, AuctionAck, Task, Bid, Winner, ScheduledTasks
from mrta.srv import TerminateRobot

cur_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(cur_dir + "/.."))
import utils

class PIA(object):

    def __init__(self, p_graph, robots, logger):        
        
        self.logger = logger
        self.p_graph = p_graph
        self.robots = robots

        rospy.init_node('auctioneer')
        self._auction_pub = rospy.Publisher('/auction', AuctionRequest, queue_size=10)
        self._winner_pub = rospy.Publisher('/winner', Winner, queue_size=10)        
        self._auction_ack_sub = rospy.Subscriber('/auction_ack', AuctionAck, self.auction_ack_callback)    
        self._bid_sub = rospy.Subscriber('/bid', Bid, self.bid_callback)
        self._scheduled_tasks_sub = rospy.Subscriber('/scheduled_tasks', ScheduledTasks, self.scheduled_tasks_callback)
        self._terminate_robot_srvs = []
        for robot in self.robots:
            terminate_robot = rospy.ServiceProxy(robot.name + '/terminate', TerminateRobot)
            self._terminate_robot_srvs.append(terminate_robot)

        self._auc_id = 1                
        self._auc_acks_flag = { robot.id : False for robot in self.robots }
        self._all_acks_received = False                
        self._auction_finished = False
        self._bids = { robot.id : (-1, None) for robot in self.robots }
        self._scheduled_tasks_flag = { robot.id : 0 for robot in self.robots }        
        self._tasks_preconditions = {}

    def allocate_tasks(self):
        n = self.p_graph.size()

        r = rospy.Rate(1)

        while not rospy.is_shutdown() :    

            ts = self.p_graph.scheduled_nodes
            tf = self.p_graph.first_layer
            tl = self.p_graph.second_layer

            self.logger.debug("{0} tasks have been scheduled.".format(len(ts)))
            self.logger.debug("First layer contains {0} tasks.".format(len(tf)))
            self.logger.debug("Second layer contains {0} tasks.".format(len(tl))) 

            if len(ts) >= n:
                self.logger.debug("All tasks have been allocated.")                
                for terminate_robot_srv in self._terminate_robot_srvs:
                    terminate_robot_srv()      
                return          

            c = max([ vertex.priority for vertex in tl ]) if tl else 0
            t_auct = set([ vertex.task for vertex in tf if vertex.priority > c ])        
            
            for v in tf:
                self.logger.debug("First layer: Task {0} with priority {1}".format(v.task.id, v.priority))

            for v in tl:
                self.logger.debug("Second layer: Task {0} with priority {1}".format(v.task.id, v.priority))

            self.logger.debug("Auction {0} started with {1} tasks.".format(self._auc_id, len(t_auct)))            
            
            while not self._auction_finished :
                if self._all_acks_received:                    
                    continue
                
                self.logger.debug("Publishing tasks")
                msg = self.create_auction_msg(self._auc_id, t_auct)
                self._auction_pub.publish(msg)

                r.sleep()    

            self.logger.debug("Auction {0} finished".format(self._auc_id))
            for t in t_auct:
                self.logger.info("Updating precedence graph")
                pc = self.p_graph.update(t)
                for x in pc:
                    self.logger.debug("Precondition for task {0} is {1}".format(x.id, pc[x]))
                for k,v in pc.iteritems():
                    self._tasks_preconditions[k] = v       
            
            self._auc_id += 1                                     
            self._auction_finished = False

    def create_auction_msg(self, _id, tasks):
        task_msgs = []
        pc = []

        for task in tasks:
            task_msgs.append(utils.create_task_msg(task))        
            pc.append(self._tasks_preconditions[task] if task in self._tasks_preconditions else 0)

        msg = AuctionRequest()
        msg.id = _id
        msg.tasks = task_msgs                  
        msg.PC = pc

        return msg

    def bid_callback(self, msg):        
        if msg.bid:
            self.logger.debug("Robot {0} has bid {1}".format(msg.robot_id, msg.bid))
            self._bids[msg.robot_id] = (msg.bid, utils.create_task(msg.task))
        
        remaining_robots = [ 1 for _, bid in self._bids.iteritems() if bid[0] == -1 ]
        
        #if all bids are received
        if len(remaining_robots) == 0:
            self.logger.info("All bids received")   
            robot_id, task = self._calc_winner()
            self._bids = { robot.id : (-1, None) for robot in self.robots }
            self._send_winner(msg.auc_id, robot_id, task)
    
    def auction_ack_callback(self, msg):        

        self._auc_acks_flag[msg.robot_id] =  True

        remaining_robots = [ 1 for _, f in self._auc_acks_flag.iteritems() if f == False ]

        if len(remaining_robots) == 0:
            self._all_acks_received = True
            self.logger.info("All acks received")

    def scheduled_tasks_callback(self, msg):
        self.logger.debug("Received scheduled tasks from robot {0}".format(msg.robot_id))
        tasks = utils.create_tasks(msg.tasks)
        for task in tasks:
            self.logger.debug("{0}".format(str(task)))
        robot_id = msg.robot_id

        self.p_graph.update_tasks(tasks)
        self._scheduled_tasks_flag[robot_id] = 1

        remaining_robots = [ 1 for _, f in self._scheduled_tasks_flag.iteritems() if f == 0 ]

        #if scheduled tasks from all robots are received         
        if len(remaining_robots) == 0:            
            self.logger.info("All scheduled tasks received. Reseting parameters")
            self._reset_params_for_new_auction()

    def _calc_winner(self):
        win_bid = float('inf')
        win_robot_id = None
        for robot_id, bid in self._bids.iteritems():
            if bid[0] < win_bid:                
                win_robot_id = robot_id
                win_bid = bid[0]
        
        win_task = self._bids[win_robot_id][1]
        return win_robot_id, win_task

    def _send_winner(self, auc_id, robot_id, task):
        winner_msg = Winner()
        winner_msg.auc_id = auc_id
        winner_msg.robot_id = robot_id
        winner_msg.task = task 

        self.logger.info("Sending winner: Auc id: {0}, Robot id: {1}, task: {2}".format(winner_msg.auc_id, winner_msg.robot_id, winner_msg.task))
        self._winner_pub.publish(winner_msg)

    def _reset_params_for_new_auction(self):
            self._auc_acks_flag = { robot.id : False for robot in self.robots }
            self._all_acks_received = False
            self._auction_finished = True