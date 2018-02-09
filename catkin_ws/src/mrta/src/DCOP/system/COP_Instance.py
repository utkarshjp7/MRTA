# coding=utf-8

'''
Created on 19 apr 2017

@author: Andrea Montanari

COP class is a Representation of a Constraint Optimization Problem.
'''

import sys, os

sys.path.append(os.path.abspath('../maxsum/'))
sys.path.append(os.path.abspath('../Graph/'))

from FactorGraph import FactorGraph


class COP_Instance:
    
    '''
        factorgraph's COP (agents, functions, variables)
    '''
    factorgraph = None

    '''
        list of agents in COP
    '''
    agents = list() 
     
    def __init__(self, nodevariables, nodefunctions, agents):
        '''
            nodevariables: COP's variables
            nodefunctions: COP's functions
            agents: COP's agents
        '''
        self.factorgraph = FactorGraph(nodevariables, nodefunctions)
        self.agents = agents
        
    def actualValue(self):
        '''
            Calculate actual value of COP adding all functions's values (utility function) 
            of factorgraph
        '''
        value = 0.0
        actualFValue = 0.0
        
        for nf in self.factorgraph.getNodeFunctions():
            actualFValue = nf.actualValue()
            '''
                not consider the unary functions (fictitious) but only the binary functions
            '''
            if((nf.getFunction().parametersNumber()) > 1):
                value = value + actualFValue
        
        return value
        
    def getFactorGraph(self):
        '''
            returns Cop's factorgraph
        '''
        return self.factorgraph
    
    def setFactorGraph(self, factorgraph):
        '''
            factorgraph: FactorGraph
            Set COP's factorgraph with factorgraph
        '''
        self.factorgraph = factorgraph
        
    def getAgents(self):
        '''
            returns Cop's agents
        '''
        return self.agents
    
    def setAgents(self, agents):
        '''
            agents: list of the agents
            Set COP's agents with agents
        '''
        self.agents = agents
        
    def addAgent(self, agent):
        '''
            agent: new Agent in COP
            Add agent to the list of agents
        '''
        return self.agents.append(agent)
    
    def getNodeFunctions(self):
        '''
            returns Cop's functions
        '''
        return self.factorgraph.getNodeFunctions()
    
    def addNodeFunction(self, nf):
        '''
            nf: new NodeFunction in COP
            Add nf to the list of functions
        '''
        return self.factorgraph.addNodeFunction(nf)
    
    def getNodeVariables(self):
        '''
            returns Cop's variables
        '''
        return self.factorgraph.getNodeVariables()
    
    def addNodeVariable(self, nv):
        '''
            nv: new NodeVariable in COP
            Add nv to the list of variables
        '''
        return self.factorgraph.addNodeVariable(nv)
    
    def setPostService(self, ps):
        '''
            ps: PostService operator
            for each agent in COP sets the operator that manages
            the sending and receiving of messages
        '''
        for agent in self.agents:
            agent.setPostService(ps)
            
    def setOperator(self, op):
        '''
            op: MaxSumOperator
            for each agent in COP sets the operator that executes 
            maxSum algorithm (max, sum)
        '''
        for agent in self.agents:
            agent.setOp(op)
             
    def setPostServiceAndOperator(self, ps, op):
        '''
            ps: PostService operator
            op: MaxSumOperator
            for each agent in COP sets the operator that manages
            the sending and receiving of messages and sets the operator 
            that executes maxSum algorithm (max, sum)
        '''
        for agent in self.agents:
            agent.setPostService(ps)
            agent.setOp(op)
            
    def status(self):
        status = ""
        
        actualValue = self.actualValue()
        status = status + str(actualValue) + ";"
        
        for agent in self.getAgents():
            for variable in agent.getVariables():
                status = status + str(variable.toString()) + "=" + str(variable.getStateArgument().toString()) + ";"
                
        return status
        
