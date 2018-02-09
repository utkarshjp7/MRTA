# coding=utf-8

'''
Created on 20 apr 2017

@author: Andrea Montanari

Agent controls variables in a COP problem instance.
Agent can control one or more variables or functions
'''

import sys, os
import datetime

sys.path.append(os.path.abspath('../Graph/'))


class Agent:
    
    '''
        kind of maxSumOperator: Max, Sum
    '''
    op = None
        
    '''
        PostService to send and retrieve messages. Used by the Nodes.
    '''
    postservice = None
    
    '''
        agent's Id
    '''
    agent_id = 0
    
    '''
        NodeVariables controlled by the agent
    '''
    variables = list()
    
    '''
        NodeFunctions controlled by the agent
    '''
    functions = list()
    
    report = ""
    
    
    def __init__(self, agent_id):
        '''
            agent_id: agent's ID
            variables: list of variables managed by Agent
            functions: list of functions managed by Agent
        '''    
        self.agent_id = agent_id
        self.variables = list()
        self.functions = list()
        self.report = ""


    def setOp(self, op):
        '''
            op: MaxSum operator
            Set Agent's maxSumOperator
        ''' 
        self.op = op
        
    
    def getPostService(self):
        '''
           returns Agent's maxSumOperator
        ''' 
        self.postservice
        
    def setPostService(self, postservice):
        '''
            postservice: maxSumOperator
            Set Agent's operator with postservice
        ''' 
        self.postservice = postservice 
        
    def setReport(self, report):
        self.report = report
    
    def getReport(self):
        return self.report
    
    def getVariables(self):
        '''
            returns variables managed by Agent
        '''
        return self.variables
    
    def getFunctions(self):
        '''
            returns functions managed by Agent
        '''
        return self.functions
    
    def getFunctionsOfVariable(self, x):
        '''
            x: NodeVariable
            returns functions have x variable
        '''
        return x.getNeighbour()
    
    def getVariablesOfFunction(self, f):
        '''
            f: NodeFunction
            returns variables in f function
        '''
        return f.getNeighbour()
    
    def setFunctions(self, functions):
        '''
            functions: functions managed by Agent
            Set functions managed by Agent with functions
        ''' 
        self.functions = functions
    
    def setVariables(self, variables):
        '''
            variables: variables managed by Agent
            Set variables managed by Agent with variables
        ''' 
        self.variables = variables
        
    def addNodeVariable(self, nodevariable):
        '''
           nodevariable: NodeVariable
           Add nodevariable to variables managed by Agent 
        '''
        self.variables.append(nodevariable)
        
    def addNodeFunction(self, nodefunction):
        '''
           nodefunction: NodeFunction
           Add nodefunction to functions managed by Agent 
        '''
        self.functions.append(nodefunction)
        
    def toString(self):
        return 'Agent_', self.agent_id;
    
    def getId(self):
        '''
            returns Agent's Id
        '''
        return self.agent_id
    
    
    def setVariableArgumentFromZ(self, x):
        '''
            x: NodeVariable
            Set the NodeVariable x value as the argMax of Z-message
        '''
        x.setStateIndex(self.op.argOfInterestOfZ(x, self.postservice))
        
    def updateVariableValue(self):
        '''
            For each variable managed by Agent,set the NodeVariable x value as the 
            argMax of Z-message
        '''
        self.report = ""
        for x in self.getVariables():
            self.setVariableArgumentFromZ(x)  
   
    def resetIds(self):
        self.agent_id = -1
        
            
    def sendQMessages(self):
        '''
            Send Q-messages phase.
            Read each RMessage by functions neighbors and computes new QMessage
            to send to function
        '''
        self.report = ""  
        
        # if new MessageQ is different from previous one
        atLeastOneUpdated = False
        
        self.report = self.report + "\n"
            
        for variable in self.getVariables():
            '''
                gotcha a variable, looking for its functions
            '''
            for function in self.getFunctionsOfVariable(variable):

                self.report = self.report + str(datetime.datetime.now())[:23] + "\t\tNodeVariable " + str(variable.toString()) + " -> " + str(function.toString()) + "\n"

                '''
                    got variable, function
                    op = maxSumOperator
                '''
                atLeastOneUpdated |= self.op.updateQ(variable, function, self.postservice)
                
                self.report = self.report + self.op.getReport()
                
                self.report = self.report + "\n" + "---------------------------------------------------------------------------------------------\n\n"
                
                
        return atLeastOneUpdated
    
      
    def sendRMessages(self):
        '''
            Send R-messages phase.
            Read each QMessage by variables neighbors and computes new RMessage
            to send to variable
        '''
        self.report = ""

        # if new MessageR is different from previous one
        atLeastOneUpdated = False
        
        self.report = self.report + "\n"
        
        for function in self.getFunctions():

            '''
                gotcha a variable, looking for its functions
            '''
            for variable in self.getVariablesOfFunction(function):
                
                self.report = self.report + str(datetime.datetime.now())[:23] + "\t\tNodeFunction " + str(function.toString()) + " -> " + str(variable.toString()) + "\n"
                '''
                    got variable, function
                    op = maxSumOperator
                '''
                atLeastOneUpdated |= self.op.updateR(function, variable, self.postservice)
                
                self.report = self.report + self.op.getReport()
                
                self.report = self.report + "---------------------------------------------------------------------------------------------\n"
                
        return atLeastOneUpdated
    
        
    def updateZMessages(self):
        '''
            Compute the Z-messages and set the variables to the value of argmax.
        '''
        self.report = ""
        
        for nodeVariable in self.getVariables():

            self.op.updateZ(nodeVariable, self.postservice)
            
            self.report = self.report + str(datetime.datetime.now())[:23] + "\t\t" + self.op.getReport()
       
        
    
    
