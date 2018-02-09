# coding=utf-8

'''
Created on 20 apr 2017

@author: Andrea Montanari

Class that represents a Factor Graph. 
A Factor Graph is composed of NodeFunctions and NodeVariables.
'''

import sys, os

sys.path.append(os.path.abspath('../Graph/'))

from Edge import Edge


class FactorGraph:
    
    '''
        Set of Nodes in the Factor Graph
    '''
    nodes = list()
    
    '''
        NodeVariables
    '''
    nodevariables = list()
    
    '''
        NodeFunctions
    '''
    nodefunctions = list()
    
    '''
        Edges
    '''
    edges = list()
    
        
    def __init__(self, nodevariables, nodefunctions):
        '''
            nodes: each node in the FactorGraph
            edges: FactorGraph's edges
            nodevariables: FactorGraph's nodeVariables
            nodefunctions: FactorGraph's nodeFunctions
        '''  
        self.nodes = list()
        self.edges = list()
        self.nodevariables = nodevariables
        self.nodefunctions = nodefunctions
       
        ''' 
            It adds all the edges (from f to x) to the set of edges of the factorgraph
        
        '''
        for f in self.nodefunctions:
            self.nodes.append(f)
            
            for arg in f.getNeighbour(): 
                edge = Edge(f, arg)
                self.edges.append(edge)
                
        for x in self.nodevariables:
            self.nodes.append(x)
        
    
    def getEdges(self):
        '''
            returns FactorGraph's edges
        '''
        return self.edges
    
    def getNodes(self):
        '''
            returns all FactorGraph's nodes
        '''
        return self.nodes
    
    def getNodeFunctions(self):
        '''
            returns FactorGraph's NodeFunctions
        '''
        return self.nodefunctions
    
    def getNodeVariables(self):
        '''
            returns FactorGraph's NodeVariables
        '''
        return self.nodevariables
    
    def getEdgeNumber(self):
        '''
            return the edges number of FactorGraph
        '''
        return len(self.edges)
    
    def setEdges(self, edges):
        '''
            edges: FactorGraph's edges
            Set FactorGraph's edges with edges
        '''
        self.edges = edges
    
    def setNodes(self, nodes):
        '''
            nodes: FactorGraph's nodes
            Set FactorGraph's nodes with nodes
        '''
        self.nodes = nodes
        
    def setNodeFunctions(self, nodefunctions):
        '''
            nodefunctions: FactorGraph's nodefunctions
            Set FactorGraph's nodefunctions with nodefunctions
        '''
        self.nodefunctions = nodefunctions
        
    def setNodeVariables(self, nodevariables):
        '''
            nodevariables: FactorGraph's nodevariables
            Set FactorGraph's nodevariables with nodevariables
        '''
        self.nodevariables = nodevariables
    
    def addNodeFunction(self, nf):
        '''
            nf: new NodeFunction
            Add nf to FactorGraph's NodeFunctions
        '''
        return self.nodefunctions.append(nf) & self.nodes.append(nf)
    
    def addNodeVariable(self, nv):
        '''
            nv: new NodeVariable
            Add nv to FactorGraph's NodeVariables
        '''
        return self.nodevariables.append(nv) & self.nodes.append(nv)
        
    def toString(self):
        string = ""
        for x in self.getNodeVariables():
            string = string + str(x.toString()) + " with neighbours:" + x.stringOfNeighbour() + "\n"
        for f in self.getNodeFunctions():
            string = string + str(f.toString()) + " with neighbours:" + f.stringOfNeighbour() + "\n"
        for e in self.getEdges():
            string = string + str(e.toString()) + "\n"
            
        return string
    
