# coding=utf-8

'''
Created on 22 apr 2017

@author: Andrea Montanari

This is class manages a r-message
'''

import sys, os

sys.path.append(os.path.abspath('../Graph/'))
sys.path.append(os.path.abspath('../messages/'))

from Message import Message


class MessageR(Message):
    
    def __init__(self, sender, receiver, message):
        '''
            sender: NodeFunction sender
            receiver: NodeVariable receiver
            message: list of the message's values
        '''
        Message.__init__(self, sender, receiver, message)
    
    def getReceiver(self):
        '''
            returns the receiver of the message
        '''
        return self.receiver

    def getSender(self):
        '''
            returns the sender of the message
        '''
        return self.sender

    def setReceiver(self, receiver):
        '''
            receiver: receiver of the message
            Sets the receiver of the message with receiver
        '''
        self.receiver = receiver
    
    def setSender(self, sender):
        '''
            sender: sender of the message
            Sets the sender of the message with sender
        '''
        self.sender = sender
