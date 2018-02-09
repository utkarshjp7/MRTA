# coding=utf-8

'''
Created on 19 apr 2017

@author: Andrea Montanari

Definition of class NodeArgument.
NodeArgument is a possible NodeVariable's value

'''

import sys, os
import random

sys.path.append(os.path.abspath('../Graph/'))



class NodeArgument:
    
    '''
        NodeArgument's value
    '''
    value = None


    def __init__(self, value):
        '''
            value: NodeArgument's value
        '''       
        self.value = value
        
    def getValue(self):
        '''
            returns NodeArgument's value
        '''
        return self.value


    def setValue(self, value):
        '''
            set NodeArgument's value with value
        '''
        self.value = value


    def toString(self):
        return str(self.value)
    
    def hashCode(self):
        return self.value.__hash__()
    
    def equals(self, n):
        '''
            returns True if NodeArgument's value is equal to n's value
                    else return False
        '''
        return self.getValue() == (n.getValue())
        
    