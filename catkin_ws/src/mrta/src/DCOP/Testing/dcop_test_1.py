import sys, os

cur_dir = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.abspath(cur_dir + '/../maxsum'))
sys.path.append(os.path.abspath(cur_dir + '/../solver'))
sys.path.append(os.path.abspath(cur_dir + '/../system'))
sys.path.append(os.path.abspath(cur_dir + '/../graph'))
sys.path.append(os.path.abspath(cur_dir + '/../misc'))
sys.path.append(os.path.abspath(cur_dir + '/../function'))
sys.path.append(os.path.abspath(cur_dir + '/../operation'))
sys.path.append(os.path.abspath(cur_dir + '/../messages'))

from Agent import Agent
from NodeVariable import NodeVariable
from NodeFunction import NodeFunction
from TabularFunction import TabularFunction
from NodeArgument import NodeArgument
from COP_Instance import COP_Instance
from MaxSum import MaxSum
from Max import Max
from MessageFactory import MessageFactory

def test():

    nodeVariable1 = NodeVariable(1)
    nodeVariable2 = NodeVariable(2)
    nodeVariable3 = NodeVariable(3)

    nodeVariable1.addDomain([1, -1])
    nodeVariable2.addDomain([1, -1]) 
    nodeVariable3.addDomain([1, 2, 3, 4, 5, -1, -2, -3, -4, -5])

    nodefunction1 = NodeFunction(1)
    nodefunction2 = NodeFunction(2)
    nodefunction3 = NodeFunction(3)
    nodefunction4 = NodeFunction(4)
    nodefunction5 = NodeFunction(5)

    nodefunction1.setFunction(TabularFunction())
    nodefunction2.setFunction(TabularFunction())
    nodefunction3.setFunction(TabularFunction())
    nodefunction4.setFunction(TabularFunction())
    nodefunction5.setFunction(TabularFunction())

    nodeVariable1.addNeighbour(nodefunction1)
    nodeVariable2.addNeighbour(nodefunction1)
    nodeVariable3.addNeighbour(nodefunction1)
    nodeVariable3.addNeighbour(nodefunction2)   
    nodeVariable3.addNeighbour(nodefunction3)
    nodeVariable3.addNeighbour(nodefunction4) 
    nodeVariable3.addNeighbour(nodefunction5)
     
    nodefunction1.addNeighbour(nodeVariable1)
    nodefunction1.addNeighbour(nodeVariable2)
    nodefunction1.addNeighbour(nodeVariable3)

    nodefunction2.addNeighbour(nodeVariable3)
    nodefunction3.addNeighbour(nodeVariable3)
    nodefunction4.addNeighbour(nodeVariable3)                                                
    nodefunction5.addNeighbour(nodeVariable3)


    nodefunction1.getFunction().addParametersCost([NodeArgument(1), NodeArgument(1), NodeArgument(1)], 7)
    nodefunction1.getFunction().addParametersCost([NodeArgument(-1), NodeArgument(1), NodeArgument(1)], 8)
    nodefunction1.getFunction().addParametersCost([NodeArgument(1), NodeArgument(-1), NodeArgument(1)], 5)
    nodefunction1.getFunction().addParametersCost([NodeArgument(1), NodeArgument(1), NodeArgument(-1)], 10)
    nodefunction1.getFunction().addParametersCost([NodeArgument(-1), NodeArgument(-1), NodeArgument(1)], 4)
    nodefunction1.getFunction().addParametersCost([NodeArgument(-1), NodeArgument(1), NodeArgument(-1)], 6)
    nodefunction1.getFunction().addParametersCost([NodeArgument(1), NodeArgument(-1), NodeArgument(-1)], 4)
    nodefunction1.getFunction().addParametersCost([NodeArgument(-1), NodeArgument(-1), NodeArgument(-1)], 0)    
     
    mfactory = MessageFactory()    
    #sum = Sum(mfactory) 
    op = Max(mfactory)

    rmessege = op.Op(nodefunction1, nodeVariable3, nodefunction1.getFunction(), [])

    print [rmessege.getValue(i) for i in range(rmessege.size())]

    return rmessege  

def create_DCop2():
    
    nodeVariables = list()
    nodeFunctions = list()    

    agent1 = Agent(1)
    agent2 = Agent(2)  
    agent3 = Agent(1)
    agent4 = Agent(2)  
    agents = [agent1, agent2, agent3, agent4]

    nodeVariable1 = NodeVariable(1)
    nodeVariable2 = NodeVariable(2)
    nodeVariable3 = NodeVariable(3)
    nodeVariable4 = NodeVariable(4)

    nodeVariable1.addDomain([1, -1])
    nodeVariable2.addDomain([2, -2]) 
    nodeVariable3.addDomain([1, 2, -1, -2])
    nodeVariable4.addDomain([2, -2])

    nodefunction1 = NodeFunction(1)
    nodefunction2 = NodeFunction(2)

    nodefunction1.setFunction(TabularFunction())
    nodefunction2.setFunction(TabularFunction())

    nodeVariable1.addNeighbour(nodefunction1)
    nodeVariable2.addNeighbour(nodefunction2)
    nodeVariable3.addNeighbour(nodefunction1)
    nodeVariable3.addNeighbour(nodefunction2)
    nodeVariable4.addNeighbour(nodefunction2)    

    nodefunction1.addNeighbour(nodeVariable1)
    nodefunction1.addNeighbour(nodeVariable3)
    nodefunction2.addNeighbour(nodeVariable2)
    nodefunction2.addNeighbour(nodeVariable3)
    nodefunction2.addNeighbour(nodeVariable4)                                                

    nodefunction1.getFunction().addParametersCost([NodeArgument(-1), NodeArgument(-1)], -100000)
    nodefunction1.getFunction().addParametersCost([NodeArgument(-1), NodeArgument(1)], 15)
    nodefunction1.getFunction().addParametersCost([NodeArgument(1), NodeArgument(-1)], 10)
    nodefunction1.getFunction().addParametersCost([NodeArgument(1), NodeArgument(1)], -100000)

    nodefunction2.getFunction().addParametersCost([NodeArgument(-2), NodeArgument(-2), NodeArgument(-2)], -100000)
    nodefunction2.getFunction().addParametersCost([NodeArgument(-2), NodeArgument(-2), NodeArgument(2)], 2)
    nodefunction2.getFunction().addParametersCost([NodeArgument(-2), NodeArgument(2), NodeArgument(-2)], 16)
    nodefunction2.getFunction().addParametersCost([NodeArgument(2), NodeArgument(-2), NodeArgument(-2)], 17)
    nodefunction2.getFunction().addParametersCost([NodeArgument(2), NodeArgument(2), NodeArgument(-2)], -100000)
    nodefunction2.getFunction().addParametersCost([NodeArgument(-2), NodeArgument(2), NodeArgument(2)], -100000)
    nodefunction2.getFunction().addParametersCost([NodeArgument(2), NodeArgument(-2), NodeArgument(2)], -100000)
    nodefunction2.getFunction().addParametersCost([NodeArgument(2), NodeArgument(2), NodeArgument(2)], -100000)    

    nodeVariables.append(nodeVariable1)
    nodeVariables.append(nodeVariable2)
    nodeVariables.append(nodeVariable3)
    nodeVariables.append(nodeVariable4)

    nodeFunctions.append(nodefunction1) 
    nodeFunctions.append(nodefunction2) 

    agent1.addNodeVariable(nodeVariable1)
    agent2.addNodeVariable(nodeVariable2)
    agent3.addNodeVariable(nodeVariable3)
    agent4.addNodeVariable(nodeVariable4)

    agent1.addNodeFunction(nodefunction1)  
    agent2.addNodeFunction(nodefunction2)
          
    cop = COP_Instance(nodeVariables, nodeFunctions, agents)    
    
    return cop  


def create_DCop():
    
    nodeVariables = list()
    nodeFunctions = list()
    agents = list()

    agent = Agent(0)  

    nodeVariable1 = NodeVariable(1)
    nodeVariable2 = NodeVariable(2)

    nodeVariable1.addDomain([0, 1, 2])
    nodeVariable2.addDomain([0, 2, 3]) 
                
    nodefunction0 = NodeFunction(0)
    nodefunction1 = NodeFunction(1)
    nodefunction2 = NodeFunction(2)
    nodefunction3 = NodeFunction(3)

    nodefunction0.setFunction(TabularFunction())
    nodefunction1.setFunction(TabularFunction())
    nodefunction2.setFunction(TabularFunction())
    nodefunction3.setFunction(TabularFunction())

    nodeVariable1.addNeighbour(nodefunction1)
    nodeVariable1.addNeighbour(nodefunction2)
    nodeVariable2.addNeighbour(nodefunction2)
    nodeVariable2.addNeighbour(nodefunction3)    

    nodefunction1.addNeighbour(nodeVariable1)
    nodefunction2.addNeighbour(nodeVariable1)
    nodefunction2.addNeighbour(nodeVariable2)
    nodefunction3.addNeighbour(nodeVariable2)                                                

    nodefunction1.getFunction().addParametersCost([NodeArgument(0)], 0)
    nodefunction1.getFunction().addParametersCost([NodeArgument(1)], 10)
    nodefunction1.getFunction().addParametersCost([NodeArgument(2)], 0)
    
    nodefunction2.getFunction().addParametersCost([NodeArgument(0), NodeArgument(0)], 0)
    nodefunction2.getFunction().addParametersCost([NodeArgument(0), NodeArgument(3)], 0)
    nodefunction2.getFunction().addParametersCost([NodeArgument(1), NodeArgument(0)], 0)
    nodefunction2.getFunction().addParametersCost([NodeArgument(1), NodeArgument(3)], 0)
    nodefunction2.getFunction().addParametersCost([NodeArgument(0), NodeArgument(2)], 4)
    nodefunction2.getFunction().addParametersCost([NodeArgument(1), NodeArgument(2)], 4)
    nodefunction2.getFunction().addParametersCost([NodeArgument(2), NodeArgument(0)], 5)
    nodefunction2.getFunction().addParametersCost([NodeArgument(2), NodeArgument(3)], 5)
    nodefunction2.getFunction().addParametersCost([NodeArgument(2), NodeArgument(2)], 17)
    
    nodefunction3.getFunction().addParametersCost([NodeArgument(0)], 0)
    nodefunction3.getFunction().addParametersCost([NodeArgument(2)], 0)
    nodefunction3.getFunction().addParametersCost([NodeArgument(3)], 8)

    nodeVariables.append(nodeVariable1)
    nodeVariables.append(nodeVariable2)
    nodeFunctions.append(nodefunction0)
    nodeFunctions.append(nodefunction1) 
    nodeFunctions.append(nodefunction2) 
    nodeFunctions.append(nodefunction3) 

    agent.addNodeVariable(nodeVariable1)
    agent.addNodeVariable(nodeVariable2)
    agent.addNodeFunction(nodefunction0)  
    agent.addNodeFunction(nodefunction1)
    agent.addNodeFunction(nodefunction2)
    agent.addNodeFunction(nodefunction3)
    agents.append(agent)
          
    cop = COP_Instance(nodeVariables, nodeFunctions, agents)    
    
    return cop  

if __name__ == "__main__":
    
    cop = create_DCop2()
    ms = MaxSum(cop, "max")
    ms.setUpdateOnlyAtEnd(False) 
    ms.setIterationsNumber(3)
    ms.solve_complete()

    result = ms.get_results2()

    for variable in result:        
        print str(variable) + ": " + str(result[variable])

    #report = ms.getReport()
    #print report
    
    