# coding=utf-8

'''
Created on 08 mag 2017

@author: Andrea Montanari

Function evaluator for MaxSum
This is an abstract class that gives the implementation of evaluateMod.
Calculates the actual value of a function and it manages the cost function
'''

import sys, os

sys.path.append(os.path.abspath('../misc/'))
sys.path.append(os.path.abspath('../Graph/'))

from NodeArgumentArray import NodeArgumentArray

class FunctionEvaluator:
         
    def getNeighbour(self):
        '''
            The NodeFunction's neighbours are stored in parameters
            returns parameters
        '''
        return self.parameters
    
    def addParameter(self, x):
        '''
            x: NodeVariable to add
            Adds x to parameters of function
        '''
        self.parameters.append(x)
    
   
    def setParameters(self, parameters):
        '''
            parameters: parameters of function (list of NodeVariables)
            Sets the NodeVariable parameters of the function
        '''
        self.parameters = parameters
        
            
    def evaluate(self, params):
        '''
            params: parameters list of function (NodeVariable)
            Evalutes f over the passed values of its arguments
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
                return self.costTable[key]
            
        return 0

      
    def parametersNumber(self):
        '''
            Returns the number of parameters used by the function
        '''
        return len(self.parameters)
    
    
    def getParameter(self, index):
        '''
            index: parameter index to find
            returns the parameter at the index-th position
        '''
        return self.parameters[index]
    

    def getParameterPosition(self, x):
        '''
            x: NodeVariable to find
            returns the position of the input x
        '''
        position = self.parameters.index(x)
        
        if (position >= 0):
            return position
        else:
            print('Parameter doesn''t find!!') 
            return -1;
            
    
    def getParameters(self):
        '''
            Returns the array of parameters. Remember that order MATTERS!
        '''
        return self.parameters
    
    
    def functionArgument(self, argumentsNumber): 
        '''
            A little tricky here.
            Let's call argumentsNumber "value"
            values = { 1, 3, 2 }
            fe is a function of x1,x2,x3
            x_i = {a,b,c,d,e} foreach i
            e.g. x1[0] = a
            fe.evaluate(fe.functionArgument(values)) means:
            fe.functionArgument(values) = { b, d, c }
            fe.evaluate( { b, d, c } );
        '''
        fzArgument = list()   
          
        for i in range(len(argumentsNumber)):
            fzArgument.insert(i, self.getParameter(i).getArgument(argumentsNumber[i]))

        return fzArgument

    
    def actualValue(self): 
        '''
            Computes the value of the function on the actual value of its parameters. 
        '''
        params = list()
        for param in self.parameters:
            params.insert(self.parameters.index(param), param.getStateArgument())
        
        return self.evaluate(params)  
        
        
