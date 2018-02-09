# coding=utf-8

'''
Created on 21 apr 2017

@author: Andrea Montanari

This class manages the Content of message.
The Content is a list of the values's message
'''


class MessageContent:
    
    '''
        list of the values's message
    '''
    message = list()
    
    def __init__(self, message):
        '''
            message: Message
        '''
        self.message = message
    
    def size(self):
        '''
            returns size of the message
        '''
        return len(self.message)
    
    def getValue(self, position):
        '''
            position: index in the message
            returns the value in position of the message
        '''
        return self.message[position]
    
    def setValue(self, position, value):
        '''
            position: index in the message
            value: new value in position in the message
            sets value in position of the message
        '''
        self.message[position] = value
    
    def toString(self):
        s = "["
        for i in range(self.size()):
            if(self.getValue(i) == None):
                s = s + "None" + ","
            else:
                s = s + str(self.getValue(i)) + ","
        s = s + "]"
        
        return s
       
