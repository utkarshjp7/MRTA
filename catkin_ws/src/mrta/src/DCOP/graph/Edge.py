# coding=utf-8

'''
Created on 19 apr 2017

@author: Andrea Montanari

Representation of an edge of the Factor Graph.
Since in a Factor Graph edges are only from NodeFunction to NodeVariable (or viceversa, they are undirected link),
an edge source is a NodeFunction and the destination is a NodeVariable.
'''

import sys, os

sys.path.append(os.path.abspath('../Graph/'))


class Edge:
    
    '''
        Edge's source, always a NodeFunction.
    '''
    source = None
    
    '''
         Edge's destination, always a NodeVariable.
    '''
    dest = None
    
    def __init__(self, source, dest):  
        '''
            source: NodeFunction
            dest: NodeVariable
        '''       
        self.source = source
        self.dest = dest
    
    
    def toString(self):
        return 'Edge{source=', self.source.toString() , 'dest=', self.dest.toString() , '}'

    
    def getSource(self):
        '''
            returns NodeVariable source
        '''
        return self.source
    
    
    def getDest(self):
        '''
            returns NodeFunction destination
        '''
        return self.dest
    
    
    def setSource(self, source):
        '''
            set Edge's source with source
        '''
        self.source = source
        
        
    def setDest(self, dest):
        '''
            set Edge's destination with dest
        '''
        self.dest = dest
        
