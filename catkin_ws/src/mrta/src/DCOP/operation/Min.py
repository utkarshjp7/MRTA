# coding=utf-8

'''
Created on 08 mag 2017

@author: Andrea Montanari

This class implements all the necessary methods to perform a correct execution 
of MinSum
'''

import sys, os

sys.path.append(os.path.abspath('../messages/'))
sys.path.append(os.path.abspath('../function/'))


class Min:
    
    '''
        MessageFactoryArrayDouble for creating a new message
    '''
    factory = None
    
    def __init__(self, factory):
        '''
            factory: MessageFactoryArrayDouble
        '''
        self.factory = factory
        
        
    def computeR(self, sender, x, fe, modifierTable):
        '''
            sender: NodeFunction sender
            x: NodeVariable receiver
            fe: FunctionEvaluator, function evaluator of MinSum 
            modifierTable: list of QMessage (x -> f)
            Computes the r message from a function f to a variable x
        '''  
        # calculates minValue of RMessage based on QMessages received
        minCost = fe.minimizeWRT(x, modifierTable)
        
        return self.factory.getMessageR(sender, x, minCost)
        
        
    def Op(self, sender, x, fe, qmessages):
        '''
            sender: NodeVariable sender
            x: NodeFunction receiver
            fe: FunctionEvaluator, function evaluator of MinSum 
            qmessages: list of qmessages (x -> f)
            Creates a table [variable sender -> qmessage]
        '''
        modifierTable = dict()
        
        for message in qmessages:
            modifierTable[message.getSender()] = message
            
        return self.computeR(sender, x, fe, modifierTable) 
        
        
    def minimizeMod(self, min, numberOfValues, x, xIndex, fe, modifierTable):
        '''
            min: list of mins values
            numberOfValues: list of parameters to maximize
            x: size of variable
            xIndex: index of variable to minimize
            fe: FunctionEvaluator, function evaluator of MinSum 
            modifierTable: table [sender -> message]
        '''
        cost = 0
        
        for xParamIndex in range(x.size()):
            numberOfValues[xIndex] = xParamIndex
            
            ''' NOW it's pretty ready
                this is the part where it is minimized
            '''
            cost = (fe.evaluateMod(fe.functionArgument(numberOfValues), modifierTable))
            
            if (min[xParamIndex] == None):
                min[xParamIndex] = cost
            elif (cost < min[xParamIndex]):
                min[xParamIndex] = cost
                
        return min
    

    def argOfInterestOfZ(self, z):
        '''
            z: array of summarized r-messages
            Given Z, it gives back the argmin (index of min)
        '''
        argmin = 0
        minvalue = z.getValue(0)
                
        for index in range(0, z.size() - 1):
  
            if (minvalue > z.getValue(index + 1)):
                argmin = index + 1
                minvalue = z.getValue(index + 1) 
                        
        return argmin
        
        
