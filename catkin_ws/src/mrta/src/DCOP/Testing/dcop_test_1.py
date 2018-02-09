import sys, os

sys.path.append(os.path.abspath('../maxsum/'))
sys.path.append(os.path.abspath('../solver/'))
sys.path.append(os.path.abspath('../system/'))
sys.path.append(os.path.abspath('../graph/'))
sys.path.append(os.path.abspath('../misc/'))
sys.path.append(os.path.abspath('../function/'))

from Agent import Agent
from NodeVariable import NodeVariable
from NodeFunction import NodeFunction
from TabularFunction import TabularFunction
from NodeArgument import NodeArgument
from COP_Instance import COP_Instance
from MaxSum import MaxSum

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
                                            
    nodefunction1.getFunction().setParameters([nodeVariable1]) 
    nodefunction2.getFunction().setParameters([nodeVariable1, nodeVariable2])
    nodefunction3.getFunction().setParameters([nodeVariable2])

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
    cop = create_DCop()
    ms = MaxSum(cop, "max")
    ms.setUpdateOnlyAtEnd(False) 
    ms.setIterationsNumber(2)
    ms.solve_complete()

    report = ms.getReport()
    print report