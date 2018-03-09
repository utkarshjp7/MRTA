# coding=utf-8

'''
Created on 08 mag 2017

@author: Andrea Montanari

This class implements all the necessary methods to perform a correct execution 
of MaxSum
'''

import sys, os
import operator

sys.path.append(os.path.abspath('../messages/'))
sys.path.append(os.path.abspath('../function/'))

class Max:
    
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
            fe: FunctionEvaluator, function evaluator of MaxSum 
            modifierTable: list of QMessage (x -> f)
            Computes the r message from a function f to a variable x
        '''
        # calculates maxValue of RMessage based on QMessages received
        maxCost = fe.maximizeWRT(x, modifierTable, sender)
    
        return self.factory.getMessageR(sender, x, maxCost)
        
        
    def Op(self, sender, x, fe, qmessages):
        '''
            sender: NodeVariable sender
            x: NodeFunction receiver
            fe: FunctionEvaluator, function evaluator of MaxSum 
            qmessages: list of qmessages (x -> f)
            Creates a table [variable sender -> qmessage]
        '''
        modifierTable = dict()
        
        for message in qmessages:
            modifierTable[message.getSender()] = message

        return self.computeR(sender, x, fe, modifierTable)   
        
        
    def maximizeMod(self, maxes, numberOfValues, x, xIndex, fe, modifierTable):
        '''
            maxes: list of maxes values
            numberOfValues: list of parameters to maximize
            x: size of variable
            xIndex: index of variable to maximize
            fe: FunctionEvaluator, function evaluator of MaxSum 
            modifierTable: table [sender -> message]
        '''
        cost = 0
        
        for xParamIndex in range(x.size()):
            numberOfValues[xIndex] = xParamIndex
            
            ''' 
                NOW it's pretty ready
                this is the part where it is maximized
            '''
            cost = (fe.evaluateMod(fe.functionArgument(numberOfValues), modifierTable))
            
            if (maxes[xParamIndex] == None):
                maxes[xParamIndex] = cost
            elif (cost > max[xParamIndex]):
                maxes[xParamIndex] = cost
                
        return maxes
    
    
    def argOfInterestOfZ(self, z):
        '''
            z: array of summarized r-messages
            Given Z, it gives back the argmax (index of max)
        '''
        argmax = 0

        if len(z) > 0:
            argmax = max(z.iteritems(), key=operator.itemgetter(1))[0]
                                
        return argmax
        
        
