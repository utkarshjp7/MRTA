# coding=utf-8

'''
Created on 10 mag 2017

@author: Andrea Montanari

Representation of parameters of the cost function.
Parameters are contained in a list (NodeArguments).
'''

import sys, os

sys.path.append(os.path.abspath('../Graph/'))


class NodeArgumentArray:
    
    '''
        list of parameters (NodeArguments) of the function
    '''
    data = None
    
    def __init__(self, params):
        '''
            params: list of the parameters (NodeArguments)
        '''
        self.data = list()
        for i in range(params.__len__()):
            self.data.append(params[i])            
                
    def toString(self):
        array = "NodeArgumentArray{" + "data=["
        for i in range(self.data.__len__()):
            array = array + self.data[i].toString() + ","
            
        array = array + "]}"
        
        return array
    
    def getArray(self):
        '''
            returns list of the parameters (NodeArguments)
        '''
        return self.data    
    
    def hashCode(self):
        hash = 0
        for nodeArgument in self.data:
            hash = hash + nodeArgument.hashCode()
        
        return hash        
    
    
        
