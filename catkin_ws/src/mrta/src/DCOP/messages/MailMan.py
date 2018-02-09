# coding=utf-8

'''
Created on 21 apr 2017

@author: Andrea Montanari

This class implements the Post Service in local mode.
It manages: sending and receiving the messages
'''

import sys, os
from collections import defaultdict

sys.path.append(os.path.abspath('../Graph/'))
sys.path.append(os.path.abspath('../messages/'))

class MailMan:
    
    '''
        interface that permits to create q-message and r-message
    '''
    factory = None
    '''
        list of qmessages
    '''
    qmessages = None
    '''
        list of rmessages
    '''
    rmessages = None
    '''
        Z-Function (sum of qmessages)
    '''
    zmessages = None
    '''
        It contains the average of the differences of the r-messages for each link
        and for each step of the MaxSum 
    '''
    rmessagesAverageDifferenceIteration = None
    
    report = ""
    
    def __init__(self, factory):
        '''
            factory: interface that permits to create q-message and r-message
        '''
        self.factory = factory
        self.rmessages = defaultdict(dict)
        self.qmessages = defaultdict(dict)
        self.zmessages = dict()
        self.rmessagesAverageDifferenceIteration = defaultdict(dict)
        
        self.report = ""
        
    def setReport(self, report):
        self.report = report
        
    def getReport(self):
        return self.report
        
    def setMessagesList(self, qmessages, rmessages):
        '''
            qmessages: list of qmessages
            rmessages: list of rmessages
            Sets the lists with qmessages and rmssages
        ''' 
        self.qmessages = qmessages
        self.rmessages = rmessages
    
    def getRmessagesAverageDifferenceIteration(self):
        '''
           returns the average of the differences of the r-messages  
        '''
        return self.rmessagesAverageDifferenceIteration
    
        
    def sendQMessage(self, x, f, value):
        '''
            x: NodeVariable sender
            f: NodeFunction receiver
            value: Message from variable to function
            Reads the qmessage (x->f) and stores it 
        '''   
        
        '''
            sets sender and receiver of the message
        '''
        value.setSender(x)      
        value.setReceiver(f)  
        
        '''
            retVal is False when actual qmessage is different from the previous one
            else it is True
        '''
        retVal = False

        '''
            c1 key isn't in the dictionary
        '''
        if x not in self.qmessages.keys():
            self.qmessages[x] = dict()
            retVal = True
        else:
            '''
                mapping c1->c2 is present
            '''                 
            for key in self.qmessages.keys():
                for obj in self.qmessages[key]:
                    if(key.getId() == x.getId()):
                        if(obj.getId() == f.getId()):
                            '''
                                verify if the mapping has value as a result 
                            '''
                            retVal = not(self.equals(self.qmessages[x][f], value))      
        
        # stores the actual qmessage (x->f)
        self.qmessages[x][f] = value  
                    
        return retVal
                
 
 
    def readQMessage(self, x, f):  
        '''
            x: NodeVariable sender
            f: NodeFunction receiver
            Reads the qmessage (x->f), if there is a new qmessage
            returns it 
        '''             
        if x not in self.qmessages.keys():
            return None
        else:            
            return self.qmessages[x][f]
        
          
    
    def sendRMessage(self, f, x, value):
        '''
            x: NodeFunction sender
            f: NodeVariable receiver
            value: Message from function to variable
            Reads the rmessage (f->x) and stores it 
        '''  
        
        '''
            sets sender and receiver of the message
        '''
        value.setSender(f)      
        value.setReceiver(x) 
        
        '''
            retVal is False when actual rmessage is different from the previous one
            else it is True
        '''
        retVal = False
        
        '''
            c1 key isn't in the dictionary
        '''
        if f not in self.rmessages.keys():
            self.rmessages[f] = dict()
            self.rmessagesAverageDifferenceIteration[f] = dict()
            retVal = True
        else:
            
            for key in self.rmessages.keys():
                for obj in self.rmessages[key]:
                    if(key.getId() == f.getId()):
                        if(obj.getId() == x.getId()):
                            '''
                                verify if the mapping has value as a result 
                            '''
                            retVal = not(self.equals(self.rmessages[f][x], value))
        
                                                                                
        '''
            calculates the difference between previous rmessage and the actual message sent,
            if it has sent a rmessage previously
        '''
        if((f in self.rmessages.keys()) & (x in self.rmessages[f])):
            average = self.difference(self.rmessages[f][x], value)
            '''
                appends the average difference between the messages in this iteration
            '''
            (self.rmessagesAverageDifferenceIteration[f][x]).append(average) 
            
        elif((f in self.rmessages.keys()) & (x not in self.rmessages[f])):                           
            average = 0
            
            self.rmessagesAverageDifferenceIteration[f][x] = list()
            '''
                if in the first iteration there isn't a previuos message, recopies 
                the rmessage received
            '''
            for i in range(value.size()):
                average = average + abs(value.getValue(i))
           
            '''
                calculates the average of the difference about this iteration
            '''
            (self.rmessagesAverageDifferenceIteration[f][x]).append(average / 3)

        # stores the actual rmessage (f->x)
        self.rmessages[f][x] = value
                
        return retVal
        
        
    def readRMessage(self, f, x):   
        '''
            f: NodeFunction sender
            x: NodeVariable receiver
            Reads the rmessage (f->x), if there is a new rmessage
            returns it 
        '''       
        if f not in self.rmessages.keys():
            return None
        else:
            return self.rmessages[f][x]  
  
        
    def readZMessage(self, x):
        '''
            x: NodeVariable respect to read the value
            Reads the value of x in Z-Function and returns it 
        '''    
        return self.zmessages[x]
    
    def setZMessage(self, x, mc):
        '''
            x: NodeVariable
            mc: MessageContent of x
            Sets the value of x with mc in Z-Function 
        '''    
        self.report = ""
        
        self.zmessages[x] = mc
        
        self.report = self.report + "ZFunction of " + str(x.toString()) + self.zmessages[x].toString() + "\n"
          
    
    def getMessageRToX(self, x):
        '''
            x: NodeVariable receiver
            List of message R addressed to x
        '''
        messages = list()
        
        for key in self.rmessages.keys():
            for message in self.rmessages[key].values():
                if(message.getReceiver() == x):
                    messages.append(message) 
        return messages
    
        
        
    def equals(self, message, mc):
        '''
            message: first Message 
            mc: second Message 
            returns True if they are equal for each value in the message
            else returns false
        '''
        
        '''for i in range(mc.size()):
            if(not(message.getValue(i) == mc.getValue(i))):
                return False'''
            
        return False 
    
    def difference(self, message, value):  
        '''
            message: first Message
            value: second Message
            returns the average of difference between message and value
        '''
        
        '''
            average of difference
            (previous message - actual message)
        '''
        average = 0
        
        for i in range(message.size()):
            
            average = average + (abs(message.getValue(i) - value.getValue(i)))
            
        return (average / 3)
            
        
