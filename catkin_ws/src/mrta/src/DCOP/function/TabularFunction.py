# coding=utf-8

'''
Created on 09 mag 2017

@author: Andrea Montanari

Tabular Function, implementation of abstract Function Evaluator.
Used for discrete functions.
This class manages the cost functions and the maximization/minimization
'''

import sys, os

sys.path.append(os.path.abspath('../function/'))
sys.path.append(os.path.abspath('../misc/'))

from FunctionEvaluator import FunctionEvaluator
from NodeArgumentArray import NodeArgumentArray
from itertools import product 

class TabularFunction(FunctionEvaluator):
    '''
        Correspondence between parameters and function values.
        The parameters are nodeVariables and the values are costs [NodeVariable -> cost]
    '''
    costTable = dict()
    '''
        list of parameters of cost function (NodeVariables)
    '''
    parameters = list()
    '''
        minimun cost of function 
    '''
    minCost = None
    '''
        maximun cost of function 
    '''
    maxCost = None
    
    report = ""
    
    
    def __init__(self):
        self.parameters = list()
        self.costTable = dict()
        self.minCost = None
        self.maxCost = None
        
        self.report = ""
        
    def setReport(self, report):
        self.report = report
        
    def getReport(self):
        return self.report    
    
    def searchKey(self, params):
        '''
            params: parameters list, key of the function cost
            looks for params in the function 
            if it finds it returns the key else returns -1
        '''
        for key in self.costTable.keys():
            
            count = 0
            
            '''
                the parameters -> key
            '''
            array = key.getArray()
            
            for i in range(len(array)):
                if ((array[i].getValue() == (params[i].getValue()))):
                    count = count + 1
                    
            if(count == len(params)):
                return key
        
        '''
            there isn't the params in the function
        '''        
        return -1
                    
        
    def addParametersCost(self, params, cost):
        '''
            params: key of cost function (list of NodeVariables)
            cost: cost function with params
            Saves the function value for NodeArgument[] of parameter.
            The params become the key of the cost table. 
        '''
        
        '''
            if there isn't the association parameters - cost
        '''
        if self.searchKey(params) == -1:
            nodeargumentarray = NodeArgumentArray(params)
            self.costTable[nodeargumentarray] = cost
        else:
            '''
                update the cost
            '''
            key = self.searchKey(params)
            self.costTable[key] = cost   
       
        '''
            update the minimun cost
        '''
        if (self.minCost == None):
            self.minCost = cost
        elif(cost < self.minCost):
            self.minCost = cost
            
        '''
            update the maximun cost
        '''
        if (self.maxCost == None):
            self.maxCost = cost
        elif(cost > self.maxCost):
            self.maxCost = cost 
                

    def entryNumber(self):
        '''
            How much values does this function have?
        '''
        return len(self.costTable)
    
    def getCostValues(self):
        '''
            returns the costs of table
        '''
        return self.costTable.values()
    
    def clearCosts(self):
        '''
            clears the cost function
        '''
        self.costTable = dict()
        
    
    def evaluateMod(self, params, modifierTable):
        '''
            params: parameters to evalute
            modifierTable: cost function
            This method evaluates the function when a list of qmessages are given
        '''
        
        '''
            if modifierTable is empty
        '''
        if(len(modifierTable) == 0):
            return self.evaluate(params)      
        
        cost = self.evaluate(params)
        
        indexOfModifier = -15
                    
        for nodeVariable in modifierTable:
            
            indexOfModifier = 0
            if params[self.getParameterPosition(nodeVariable)] < 0:
                indexOfModifier = 1
                        
            cost = cost + modifierTable[nodeVariable].getValue(indexOfModifier)

        return cost
    
    
    def maximizeWRT(self, x, modifierTable, sender):
        '''
            x: variable respect to maximize
            modifierTable: cost function
            calls the maximization function
        '''
        return self.maxminWRT("max", x, modifierTable, sender)


    def minimizeWRT(self, x, modifierTable):
        '''
            x: variable respect to minimize
            modifierTable: cost function
            calls the minimization function
        '''
        return self.maxminWRT("min", x, modifierTable)
    
    
    def maxmin(self, op, maxes, functionArgument, x, xIndex, modifierTable, sender):   
        '''
            op: max/min
            maxes: actual maxes about variable
            functionArgument: actual parameters
            x: variable to maximize
            xIndex: index of x in cost function
            modifierTable: cost function
            Calculates the maxes with functionArgument respect x
        '''     
        if(op == "max"):
            cost = float("-Infinity")
        elif(op == "min"):
            cost = float("+Infinity")
                     
        indexes = [x.getIndexOfValue(sender.function_id), x.getIndexOfValue(sender.function_id) + x.size()/2]

        for i, xParamIndex in enumerate(indexes):

            functionArgument[xIndex] = xParamIndex
          
            '''            
               NOW it's pretty ready
               this is the part where it is maximized
            '''
            if(modifierTable == None):
                cost = self.evaluate(self.functionArgument(functionArgument))
            else:
                cost = (self.evaluateMod(self.functionArgument(functionArgument), modifierTable))
    
            if(op == "max"):
                if (maxes[i] < cost):
                    maxes[i] = (cost)
            elif(op == "min"):
                if (maxes[i] > cost):
                    maxes[i] = (cost)

        return maxes

    
    def maxminWRT(self, op, x, modifierTable, sender):
        '''
            op: max/min
            x: variable respect to maximize
            modifierTable: cost function
            Calculates the max value on function respect x
        '''
        
        '''
            index of x in function
        '''
        xIndex = self.getParameterPosition(x)     
        
        '''
            number of parameters of f
        '''
        fzParametersNumber = self.parametersNumber()
        
        '''
            The i-th position of list will be the number of possible values of 
            the i-th argument of f. At the position of x, there will be 
            only one value available
        '''
        numberOfValues = list()
        
        '''
            the array filled with variable value positions that's gonna be evaluated
        '''
        functionArgument = list()
        
        '''
            set to zero functionArgument
        '''
        for i in range(fzParametersNumber):
            if i != xIndex:
                pos_index = self.getParameter(i).getIndexOfValue(sender.function_id)
                neg_index = self.getParameter(i).getIndexOfValue(-sender.function_id)        
                functionArgument.append([pos_index, neg_index])
        
        functionArguments = product(*functionArgument)
            
        '''
            maximization array, wrt x possible values
        '''
        maxes = list()
        
        for index in range(2):
            if(op == "max"):
                maxes.append(float("-Infinity"))
            elif(op == "min"):
                maxes.append(float("+Infinity"))
                       
                         
        for i in range(fzParametersNumber):
            numberOfValues.append(2)
            
        numberOfValues[xIndex] = 1
        
        imax = len(numberOfValues) - 1  
        
        i = imax  

        for argument in functionArguments:
            argument = list(argument)
            argument.insert(xIndex, 0)
            maxes = self.maxmin(op, maxes, argument, x, xIndex, modifierTable, sender)





        
        return maxes 
    
    
    def toString(self):
        ris = "Function evaluator with " + str(self.entryNumber()) + " entries\n"
        ris = ris + "NodeVariable used: " 
        
        for i in range(self.parameters.__len__()):
            ris = ris + str(self.parameters[i].toString()) + " "
            
        ris = ris + "\n"
            
        for entry in self.costTable:
            ris = ris + "[ "
            nodeArguments = entry.getArray()
            
            for i in range(len(nodeArguments)):
                ris = ris + str(nodeArguments[i].toString()) + " "
                
            ris = ris + "Value: " + str(self.costTable[entry]) + " ]\n"
         
        return ris       
    
