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
        self._auc_acks_flag = { (i+1) : False for i in range(self._robot_count) }
        self._all_acks_received = False        
        self._auction_finished = False
        self._bids = { (i+1) : (-1, None) for i in range(self._robot_count) }
        self._scheduled_tasks_flag = { (i+1) : 0 for i in range(self._robot_count) }

        self._tasks_preconditions = {}

    def start_allocation(self):
        n = self.p_graph.size()

        r = rospy.Rate(1)

        while not rospy.is_shutdown() :    

            ts = self.p_graph.scheduled_nodes
            tf = self.p_graph.first_layer
            tl = self.p_graph.second_layer

            print ""
            print str(len(ts)) + " tasks have been scheduled."
            print "First layer contains " + str(len(tf)) + " tasks."
            print "Second layer contains " + str(len(tl)) + " tasks."
            print ""

            if len(ts) >= n:
                print "All tasks have been allocated."
                break

            c = max([ vertex.priority for vertex in tl ]) if tl else 0
            t_auct = set([ vertex.task for vertex in tf if vertex.priority > c ])        
            
            for v in tf:
                print "First layer: Task " + str(v.task.id) + " with priority " + str(v.priority)

            for v in tl:
                print "Second layer: Task " + str(v.task.id) + " with priority " + str(v.priority)

            print "Auction " + str(self._auc_id) + " started with " + str(len(t_auct)) + " tasks."            
            
            while not self._auction_finished :
                if self._all_acks_received:                    
                    continue
                
                print "Publishing tasks"
                msg = self.create_auction_msg(self._auc_id, t_auct)
                self._auction_pub.publish(msg)

                r.sleep()    

            print "Auction " + str(self._auc_id) + " finished"
            for t in t_auct:
                print "Updating precedence graph"
                pc = self.p_graph.update(t)
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
            print "Robot " + str(msg.robot_id) + " has bid " + str(msg.bid)
            self._bids[msg.robot_id] = (msg.bid, utils.create_task(msg.task))
        
        remaining_robots = [ 1 for _, bid in self._bids.iteritems() if bid[0] == -1 ]
        
        #if all bids are received
        if len(remaining_robots) == 0:
            print "All bids received"   
            robot_id, task = self._calc_winner()
            self._bids = { (i+1) : (-1, None) for i in range(self._robot_count) }
            print "Sending winner"
            self._send_winner(msg.auc_id, robot_id, task)
    
    def auction_ack_callback(self, msg):        

        self._auc_acks_flag[msg.robot_id] =  True

        remaining_robots = [ 1 for _, f in self._auc_acks_flag.iteritems() if f == False ]

        if len(remaining_robots) == 0:
            self._all_acks_received = True
            print "All acks received"

    def scheduled_tasks_callback(self, msg):
        print "Received scheduled tasks from robot " + str(msg.robot_id)
        tasks = utils.create_tasks(msg.tasks)
        robot_id = msg.robot_id

        self.p_graph.update_tasks(tasks)
        self._scheduled_tasks_flag[robot_id] = 1        

        remaining_robots = [ 1 for _, f in self._scheduled_tasks_flag.iteritems() if f == 0 ]

        #if scheduled tasks from all robots are received         
        if len(remaining_robots) == 0:            
            print "All scheduled tasks received. Reseting parameters"
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
            self._auc_acks_flag = { (i+1) : False for i in range(self._robot_count) }
            self._all_acks_received = False
            self._auction_finished = True