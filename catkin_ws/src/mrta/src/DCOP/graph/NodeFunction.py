# coding=utf-8

'''
Created on 19 apr 2017

@author: Andrea Montanari

The object that implement the function is a Function Evaluator.
Each NodeFunction has a Function Evaluator.
Each NodeFunction can have NodeVariables neighbours
'''

import sys, os

sys.path.append(os.path.abspath('../Graph/'))
sys.path.append(os.path.abspath('../function/'))


class NodeFunction:
 
    '''
        The FunctionEvaluator that implements the function represented by this NodeFunction.
    '''
    functionEvaluator = None
    
    function_id = -1
        

    def __init__(self, function_id):
        '''
            function_id: NodeFunction's Id
        '''   
        self.function_id = function_id

    def setFunction(self, functionEvaluator):
        '''
            functionEvaluator: functionEvaluator's NodeFunction
            Set functionEvaluator's NodeFunction with functionEvaluator
        '''   
        self.functionEvaluator = functionEvaluator

    def getFunction(self):
        '''
            returns functionEvaluator's NodeFunction
        '''
        return self.functionEvaluator

    def addNeighbour(self, x):
        '''
            x: new NodeVariable neighbour
            Add x to neighbour list of NodeFunction
        '''
        self.functionEvaluator.addParameter(x)


    def size(self):
        '''
            return the number of arguments of the function
        '''
        return len(self.functionEvaluator.getNeighbour())


    def getNeighbour(self):
        '''
            returns the nodeFunction's neighbours
        '''
        return self.functionEvaluator.getNeighbour()


    def toString(self):
        return 'NodeFunction_', self.function_id

    def getId(self):
        '''
            returns the nodeFunction's Id
        '''
        return self.function_id

    def actualValue(self):
        '''
            returns the NodeFunction's actual value based on the Parameters's value
        '''
        return self.functionEvaluator.actualValue()

    def hashCode(self):
        return ("NodeFunction_", self.getId()).__hash__()


    def resetIds(self):
        self.function_id = -1
        
        
    def stringOfNeighbour(self):
        neighbours = ""
        for nodevariable in self.getNeighbour():
            neighbours = neighbours + str(nodevariable.toString()) + " "
        return neighbours
        
