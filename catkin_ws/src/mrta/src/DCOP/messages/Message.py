# coding=utf-8

'''
Created on 21 apr 2017

@author: Andrea Montanari

This class implements all the necessary methods to perform a Message.
'''

import sys, os

sys.path.append(os.path.abspath('../messages/'))

from MessageContent import MessageContent


class Message:
    '''
        list of the values in the message
    '''
    message = None
    '''
        sender of the message
    ''' 
    sender = None
    '''
        receiver of the message
    '''
    receiver = None
    
    def __init__(self, sender, receiver, message):
        '''
            sender: Node sender the message
            receiver: Node receiver the message
            message: list of the values in the message
        '''
        self.sender = sender
        self.receiver = receiver
        self.message = MessageContent(message)
    
    def getMessage(self):
        '''
            returns the message
        '''
        return self.message
    
    def setMessage(self, message):
        '''
            message: Message Q/R
        '''
        self.message = message
        
    def getValue(self, position):
        '''
            position: index of value in the message
            returns the content in index of the message
        '''
        return self.message.getValue(position)
    
    def setValue(self, position, value):
        '''
            position: index of value in the message
            value: new value in position
            Sets the content in index of the message with value
        '''
        return self.message.setValue(position, value)
    
    def setSender(self, sender):
        '''
            sender: sender of the message
            Sets the sender of the message with sender
        '''
        self.sender = sender
        
    def setReceiver(self, receiver):
        '''
            receiver: receiver of the message
            Sets the receiver of the message with receiver
        '''
        self.receiver = receiver
        
    def getSender(self):
        '''
            returns the sender of the message
        '''
        return self.sender
    
    def getReceiver(self):
        '''
            returns the receiver of the message
        '''
        return self.receiver
    
    def size(self):
        '''
            returns the size of the message
        '''
        return self.message.size()
    
    def toString(self):
        return 'Message from ', self.sender.toString(), ' to ' , self.receiver.toString()
    
    
