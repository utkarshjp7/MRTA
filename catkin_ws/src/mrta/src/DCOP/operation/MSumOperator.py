# coding=utf-8

'''
Created on 08 mag 2017

@author: Andrea Montanari

This class implements all the necessary methods to perform a correct execution 
of MaxSum or MinSum.
It manages the calculating of rmessages/qmessages
'''

import sys, os
import datetime

sys.path.append(os.path.abspath('../system/'))
sys.path.append(os.path.abspath('../operation/'))


class MSumOperator:
    
    '''
        maximization/minimization (Max/Min operator)
    '''
    type = None
    
    '''
        simple sum of all qmessages
    '''
    sum = None
    
    report = ""
    
    def __init__(self, sum, type):
        '''
            sum: Sum operator of all qmessages
            type: Max/Min operator
        '''
        self.sum = sum
        self.type = type
        self.report = ""
     
    def setType(self, type): 
        '''
            type: Max/Min operator
            Sets MaxSum operator with type 
        '''  
        self.type = type
        
    def setSum(self, sum):   
        '''
            sum: Sum operator of QMessages
            returns sum operator
        '''
        self.sum = sum
        
    def setReport(self, report):
        self.report = report
        
    def getReport(self):
        return self.report    
        
    
    def computeQ(self, sender, receiver, alpha, rmessages):
        '''
            sender: NodeVariable
            receiver: NodeFunction
            alpha: normalization factor of Qmessage
            It computes the q-message, given the normalization factor alpha and the list 
            of r-messages
        '''
        if len(rmessages) == 0:
            return None
        
        qmessage = self.sum.op(sender, receiver, rmessages)
        
        for i in range(0, qmessage.size()):
            qmessage.setValue(i, qmessage.getValue(i) + alpha)
            
        return qmessage  
        
        
    def computeAlpha(self, sender, receiver, rmessages):
        '''
            sender: NodeVariable
            receiver: NodeFunction
            rmessages: list of r-messages
            Computes the alpha, the normalization factor (sum of each rmessage divide the 
            domain's variable)  
        '''
        if (rmessages == None) | (len(rmessages) == 0):
            return 0
        
        qmessage = self.sum.op(sender, receiver, rmessages)
        
        alpha = qmessage.getValue(0)
        for i in range(1, qmessage.size()):
            alpha = alpha + qmessage.getValue(i)
        alpha = alpha * -1.0
        alpha = alpha / qmessage.size()
        
        return alpha
     
    
    def computeZ(self, x, rmessages):
        '''
            x: NodeVariable respect to which calculate Z function (Z is the "sum message" of each Qmessage)
            rmessages: list of r-messages to be added
            Summarize the r-messages
        '''
        if len(rmessages) == 0:
            return self.sum.nullMessage(x, None, x.size()).getMessage()
        else:
            return self.sum.op(x, None, rmessages).getMessage()
        
        
    def updateQ(self, x, f, postservice):
        '''
            x: NodeVariable receiver
            f: NodeFunction sender
            postservice: PostService to send and retrieve messages. Used by the Nodes.
            Receives r-messages from functions and It sends Qmessages
        '''
        self.report = " "
                
        rmessages = list()
        
        '''
            Q from other functions
        '''
        for function in x.getNeighbour():
            
            '''
                every new iterator value is a new index in M(i)
                considers all functions except f
            '''
            if (function.getId() != f.getId()):
                
                self.report = self.report + "\n"
                
                value = postservice.readRMessage(function, x)
                
                if(value == None):
                    self.report = self.report + str(datetime.datetime.now())[:23] + "\t\tRead rmessage from: " + str(function.toString()) + " --> rmessage: " + str(value) + "\n"
                else:
                    self.report = self.report + str(datetime.datetime.now())[:23] + "\t\tRead rmessage from: " + str(function.toString())
                    

                if(value != None):
                    '''
                        if there's a message in (f,x) add it to the list
                    ''' 
                    rmessages.append(value)                    

        
        self.report = self.report + "\n"
            
        
        for val in rmessages:
            self.report = self.report + " values:" + str(val.getMessage().toString()) + "\n"
            
        messageq = self.computeQ(x, f, self.computeAlpha(x, f, rmessages), rmessages)
                
        
        if (messageq == None):
            messageq = self.sum.nullMessage(x, function, x.size())
            
        self.report = self.report + "\n\t\t\t\t\t\t\tQMessage: "
        for i in range(x.size()):
            self.report = self.report + str(messageq.getMessage().getValue(i)) + ","
            
        self.report = self.report + "\n"     
    
        return postservice.sendQMessage(x, f, messageq) 
     
    
    def updateR(self, f, x, postservice):
        '''
            f: NodeFunction sender
            x: NodeVariable receiver
            postservice: PostService to send and retrieve messages. Used by the Nodes.
            Receives q-messages from variables and It sends Rmessages
        '''
        self.report = ""
                
        qmessages = list() 
        
        '''
            R from other variables
        '''
        for variable in f.getNeighbour():
            
            '''
                every new iterator value is a new index in M(i)
                considers all variables except f
            '''
            if (variable.getId() != x.getId()):
                
                self.report = self.report + "\n"
                
                value = postservice.readQMessage(variable, f)
                    
                if(value == None):
                    self.report = self.report + str(datetime.datetime.now())[:23] + "\t\tRead qmessage from: " + str(variable.toString()) + " --> qmessage: " + str(value) + "\n"
                else:
                    self.report = self.report + str(datetime.datetime.now())[:23] + "\t\tRead qmessage from: " + str(variable.toString()) + " --> "
                
                if(value != None):
                    '''
                        if there's a message in (x,f) add it to the list
                    '''
                    qmessages.append(value)
    
                self.report = self.report + " QMessage: "
                
                for i in range(variable.size()):
                    self.report = self.report = self.report + str(value.getMessage().getValue(i)) + ","
                    
                self.report = self.report + "\n\n"
           
        messager = self.type.Op(f, x, f.getFunction(), qmessages)

        self.report = self.report + f.getFunction().getReport()
        
        
        self.report = self.report + "\t\t\t\t\t\t\tRMessage: "
        for i in range(x.size()):
            self.report = self.report + str(messager.getMessage().getValue(i)) + ","

        self.report = self.report + "\n"
        
        return postservice.sendRMessage(f, x, messager)
    
    
    def getReportMessage(self):
        return self.postservice.getReport()
    
    
    def updateZ(self, x, ps):  
        '''
            x: NodeVariable respect to which calculate Z function(sum of all q-messages)
            Sum the incoming qmessages 
        '''      
        self.report = ""
        
        ps.setZMessage(x, self.computeZ(
                    x,
                    ps.getMessageRToX(x)
                ))
        self.report = self.report + ps.getReport()
        
        
    def argOfInterestOfZ(self, x, ps):
        '''
            x: NodeVariable respect to which maximize/minimize Z function
            ps: PostService to send and retrieve messages. Used by the Nodes.
            Implementation of arg-max/arg-min of Z
        '''    
        return self.type.argOfInterestOfZ(
                ps.readZMessage(x)
                )
            
