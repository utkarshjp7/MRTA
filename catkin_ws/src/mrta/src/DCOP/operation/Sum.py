# coding=utf-8

'''
Created on 08 mag 2017

@author: Andrea Montanari

This class manages the operation on QMessages
'''

import sys, os

sys.path.append(os.path.abspath('../Graph/'))


class Sum:
    
    '''
        MessageFactoryArrayDouble for creating a new message
    '''
    factory = None
    
    def __init__(self, factory):
        '''
            factory: MessageFactoryArrayDouble
        '''
        self.factory = factory
        
    
    def nullMessage(self, sender, receiver, size):
        '''
            sender: NodeVariable
            receiver: NodeFunction
            size: domain's variable
            Creates new QMessage with 0 for each domain's value of variable
        '''
        content = list()
        
        for i in range(size):
            content.append(0.0)
            
        return self.factory.getMessageQ(sender, receiver, content)
       
    def op(self, sender, receiver, rmessages): 
        '''
            sender: NodeFunction
            receiver: NodeVariable
            rmessages: incoming rmessages to variable
            Calculates the sum of all incoming rmessages to variable
        '''   
        if(len(rmessages) == 0):
            return None
        
        if(rmessages == None):
            return None
        
        size = rmessages[0].size()
        
        content = list()
             
        for i in range(size):
            content.append(0.0)

        for i in range(len(rmessages)):
            for j in range(rmessages[i].size()):
                content[j] = content[j] + rmessages[i].getValue(j)

        return self.factory.getMessageQ(sender, receiver, content)
        
