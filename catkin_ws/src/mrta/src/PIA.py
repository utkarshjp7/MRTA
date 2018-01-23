import rospy
import utils
from mrta.msg import AuctionRequest, AuctionAck, Task, Bid, Winner, ScheduledTasks

class PIA(object):

    def __init__(self, p_graph):
        self.p_graph = p_graph
        self._robot_count = int(rospy.get_param("/robot_count", 1))

        rospy.init_node('auctioneer')
        self._auction_pub = rospy.Publisher('/auction', AuctionRequest, queue_size=10)
        self._winner_pub = rospy.Publisher('/winner', Winner, queue_size=10)
        self._auction_ack_sub = rospy.Subscriber('/auction_ack', AuctionAck, self.auction_ack_callback)    
        self._bid_sub = rospy.Subscriber('/bid', Bid, self.bid_callback)
        self._scheduled_tasks_sub = rospy.Subscriber('/scheduled_tasks', ScheduledTasks, self.scheduled_tasks_callback)

        self._auc_id = 1        
        #self._auc_acks = { self._auc_id : { (i+1) : False for i in range(self._robot_count) } }
        self._auc_acks_flag = { (i+1) : False for i in range(self._robot_count) }

        self._all_acks_received = False
        self._auction_finished = False
        self._bids = { (i+1) : (-1, None) for i in range(self._robot_count) }
        self._scheduled_tasks_flag = { (i+1) : 0 for i in range(self._robot_count) }

        self._F = []
        self._PC = []

    def start_allocation(self):
        n = self.p_graph.size()
        ts = self.p_graph.scheduled_tasks
        tf = self.p_graph.first_layer
        tl = self.p_graph.second_layer
        th = self.p_graph.hidden_layer

        self._F = n * [0]
        self._PC = n * [0]

        r = rospy.Rate(1)

        while len(ts) < n :                            
            c = max([ vertex.priority for vertex in tl ]) if tl else 0
            t_auct = set([ vertex.task for vertex in tf if vertex.priority > c ])
            
            while not ( rospy.is_shutdown() or self._auction_finished ):
                if self._all_acks_received:
                    continue
                
                msg = self.create_auction_msg(self._auc_id, t_auct)
                self._auction_pub.publish(msg)

                r.sleep()    

            for t in t_auct:
                pc = self.p_graph.update(t)
                for k,v in pc.iteritems():
                    self._PC[k] = v        
            
            break

    def create_auction_msg(self, _id, tasks):
        task_msgs = []
        
        for task in tasks:
            task_msgs.append(utils.create_task_msg(task))

        msg = AuctionRequest()
        msg.id = _id
        msg.tasks = task_msgs
        msg.PC = self._PC

        return msg

    def bid_callback(self, msg):
        if msg.bid:
            self._bids[msg.robot_id] = (msg.bid, utils.create_task(msg.task))
        
        remaining_robots = [ 1 for _, bid in self._bids.iteritems() if bid[0] == -1 ]
        
        #if all bids are received
        if len(remaining_robots) == 0:   
            robot_id, task = self._calc_winner()
            self._bids = { (i+1) : (-1, None) for i in range(self._robot_count) }
            self._send_winner(msg.auc_id, robot_id, task)
    
    def auction_ack_callback(self, msg):        

        self._auc_acks_flag[msg.robot_id] =  True

        #ack_count = 0
        remaining_robots = [ 1 for _, f in self._auc_acks_flag.iteritems() if f == False ]

        if len(remaining_robots) == 0:
            self._all_acks_received = True

        """
        for auc_id, robots in self._auc_acks.iteritems():
            for robot_id, status in robots.iteritems(): 
                if auc_id == self._auc_id and status:
                    ack_count += 1    

        if ack_count == self._robot_count:
            self._all_acks_received = True
        """

    def scheduled_tasks_callback(self, msg):
        tasks = utils.create_tasks(msg.tasks)
        robot_id = msg.robot_id

        self.p_graph.update_tasks(tasks)
        self._scheduled_tasks_flag[robot_id] = 1        

        remaining_robots = [ 1 for _, f in self._scheduled_tasks_flag.iteritems() if f == 0 ]

        #if scheduled tasks from all robots are received 
        if len(remaining_robots) == 0:            
            self._reset_params_for_new_auction()

    def _calc_winner(self):
        win_bid = float('inf')
        win_robot_id = None
        for robot_id, bid in self._bids.iteritems():
            if bid[0] < win_bid:                
                win_robot_id = robot_id
        
        win_task = self._bids[win_robot_id][1]
        return win_robot_id, win_task

    def _send_winner(self, auc_id, robot_id, task):
        winner_msg = Winner()
        winner_msg.auc_id = auc_id
        winner_msg.robot_id = robot_id
        winner_msg.task = task 
        self._winner_pub.publish(winner_msg)

    def _reset_params_for_new_auction(self):
            self._auc_id += 1
            #self._auc_acks = { self._auc_id : { (i+1) : False for i in range(self._robot_count) } }
            self._auc_acks_flag = { (i+1) : False for i in range(self._robot_count) }
            self._all_acks_received = False