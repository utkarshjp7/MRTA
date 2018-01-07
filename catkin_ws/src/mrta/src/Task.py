import uuid

class Task(object):
    
    def __init__(self, esf, lft, duration, _id, pos_x, pos_y):
        self.id = _id
        self.esf = esf
        self.lft = lft
        self.duration = duration
        self.location = (pos_x, pos_y)        

    def __eq__(self, other):
        return self.id == other.id      

    def __str__(self):
        s = "id: " + str(self.id) + ", "
        s += "esf: " + str(self.esf) + ", "
        s += "lft: " + str(self.lft) + ", "
        s += "duration: " + str(self.duration) + "."
        return str(self.id)

    def __hash__(self):
        return hash((self.id, self.esf, self.lft, self.duration, self.location))