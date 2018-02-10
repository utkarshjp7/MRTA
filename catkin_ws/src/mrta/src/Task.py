import uuid

class Task(object):
    
    def __init__(self, est, lft, duration, _id, pos_x, pos_y, _type=None):
        self.id = _id
        self.start_time = 0
        self.finish_time = 0
        self.est = est
        self.lst = lft - duration
        self.eft = est + duration    
        self.lft = lft             
        self.duration = duration
        self.location = (pos_x, pos_y)
        self.type = _type

    def __eq__(self, other):
        return self.id == other.id      

    def __str__(self):
        s = "id: " + str(self.id) + ", "
        s += "est: " + str(self.est) + ", "
        s += "lft: " + str(self.lft) + ", "             
        s += "start_time: " + str(self.start_time) + ", "
        s += "end_time: " + str(self.finish_time) + ", "
        s += "duration: " + str(self.duration) + ", " 
        s += "location: " + str(self.location) + "."
        return str(s)

    def __hash__(self):
        return hash(self.id)

    def update_time_window(self, est, lft):
        self.est = est
        self.lst = lft - self.duration
        self.eft = est + self.duration    
        self.lft = lft  