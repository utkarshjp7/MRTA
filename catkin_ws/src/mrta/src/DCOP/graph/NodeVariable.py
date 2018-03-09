# coding=utf-8

'''
Created on 19 apr 2017

@author: Andrea Montanari

Definition of class NodeVariable
A variable can take many values (NodeArgument) from his domain and It can have 
NodeFunctions neighbours 

'''

import sys, os

sys.path.append(os.path.abspath('../Graph/'))

from NodeArgument import NodeArgument



class NodeVariable:

    '''
        represent M(i), that is the set of function nodes connected to the variable i
    '''
    neighbours = list()

    '''
        variable's id
    '''
    id_var = -1

    '''
        arraylist of the possible values of the variable represented by this node
    '''
    values = list()

    '''
        index of the actual value of this NodeVariable.
    '''
    index_actual_argument = -1
    
    '''
        color taken by variable
    '''
    color = -1

    def __init__(self, id_var):
        '''
            id_var: variable's id
        '''       
        self.id_var = id_var
        '''
            values: variable's domain
        '''
        self.values = list()
        '''
            neighbours: functions neighbours of NodeVariable
        '''
        self.neighbours = list()
        '''
            color taken by variable
        '''
        self.color = -1

    def setColor(self, color):
        '''
            set NodeVariable's color with color
        '''
        self.color = color
        
    def getColor(self):
        '''
            returns NodeVariable's color
        '''
        return self.color
        
    
    def addValue(self, v):
        '''
           Add to domain's variable a new possible value
        '''
        self.values.append(v)

    def addIntegerValues(self, number_of_values):
        '''
            number_of_values: quantity of the values to add to domain's variable
            Utility that, given a number of values, creates for this variables the corresponding NodeArguments
            E.g. x.addIntegerValues(3) means that x = { 0 | 1 | 2 }
        '''
        for i in range(number_of_values):
            nodeargument = NodeArgument(i)
            self.addValue(nodeargument)

    def addDomain(self, task_id_arr):
        for task_id in task_id_arr:
            nodeArgument = NodeArgument(task_id)
            self.addValue(nodeArgument)

    def getValues(self):
        '''
            returns the domain's variable
        '''
        return self.values

    def addNeighbour(self, x):
        '''
            x: new function neighbour of the variable
            Add new neighbour to variable
        '''
        self.neighbours.append(x)


    def removeNeighbours(self, c):
        '''
            c: functions list to remove
            Remove each function in c from function neighbours of the variable
        '''
        for f in c:
            self.neighbours.remove(f)

    
    def size(self):
        '''
            return length of the domain's variable
        '''
        return len(self.values)

    def getArgument(self, index):  
        '''
            index: NodeArgument's index to find
            returns the NodeArgument in index position
        '''
        return self.values[index]

    def getNeighbour(self):
        '''
            returns functions neighbours of the variable
        '''
        return self.neighbours

    def clearValues(self):
        '''
            clear the domain's variable
        '''
        self.values = list()


    def toString(self):
        return 'NodeVariable_', self.id_var


    def setStateIndex(self, index):
        '''
            index: index of actual value's the variable
            Set the index of actual parameter
        '''
        self.index_actual_argument = index


    def setStateArgument(self, n):
        '''
            n: actual nodeArgument of the variable
            Set the actual NodeArgument
        '''
        self.index_actual_argument = self.getIndexOfValue(n.value)


    def getStateIndex(self):
        '''
            returns the actual value's index of the variable
        '''
        return self.index_actual_argument


    def getStateArgument(self):
        '''
            returns the actual value of the variable
            if index_actual_argument is equal -1, the variable has not been set
        '''
        #if self.index_actual_argument == -1:
        #    print('The variable ', self.toString(), ' has not been set')
        return self.getArgument(self.index_actual_argument)
            
    def getId(self):
        '''
            returns the variable's Id
        '''
        return self.id_var
    
    def hashCode(self):
        return ('NodeVariabile_', self.id_var).__hash__()

    def resetIds(self):
        '''
            clear each function neighbour of the variable
        '''
        self.neighbours = list()

    def getArguments(self):
        '''
            returns each value of domain's variable
        '''
        return self.values

    def getIndexOfValue(self, value):
        '''
            value: Value to find
            it gives the position of the argument over the possible values
        '''
        index = [i for i in range(self.size()) if self.values[i].value == value]
        if len(index) > 0:
            return index[0]

    def stringOfNeighbour(self):
        neighbours = ""
        for nodefunction in self.neighbours:
            neighbours = neighbours + str(nodefunction.toString()) + " "
        return neighbours

