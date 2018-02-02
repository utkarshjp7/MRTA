
import matplotlib.pyplot as plt
import networkx as nx
import random
import math
import copy
import re

'''
#INPUT: set of tasks(time window, location, id), set of robots(initial position, each robot's speed, STN),
INPUTS: No_var, No_fac
'''

def Qmessage(G,Q,R):
    No_fac=len([n for n,attrdict in G.node.items() if attrdict['type'] == 'Factor' ])
    No_var=len([n for n,attrdict in G.node.items() if attrdict['type'] == 'Var' ])
    
    for i in range(No_var):
    # Collect Rmessages from all neighboring functions, except the Qmessage receiver
        for j in range(No_fac):
            if 'F_'+str(j) in G.neighbors('V_'+str(i)):
            #if j in [index for index, value in enumerate(variables[i]) if value==1]:
                #for k in [index for index, value in enumerate(variables[i]) if value==1]:
                q=map(int,re.findall('\d+', ''.join(G.neighbors('V_'+str(i)))))
                tmp=list(q)
                tmp.sort()
                for k in tmp:
                    if k!=j:
                        w=[sum(x) for x in zip(Q[i][j], R[k][i])]
                        Q[i][j]=w
    return Q

def Rmessage(G,Q,R,f_table):
    No_fac=len([n for n,attrdict in G.node.items() if attrdict['type'] == 'Factor' ])
    No_var=len([n for n,attrdict in G.node.items() if attrdict['type'] == 'Var' ])
    for j in range(No_fac):
        for i in range(No_var):
            #if i in [index for index, value in enumerate(factors[j]) if value==1]:
            if 'V_'+str(i) in G.neighbors('F_'+str(j)):
                #for k in [index for index, value in enumerate(factors[j]) if value==1]:
                q=map(int,re.findall('\d+', ''.join(G.neighbors('F_'+str(j)))))
                tmp=list(q)
                tmp.sort()
                for k in tmp:
                    if k!=i:
                        try:
                            sigma_Q.update({k:Q[k][j]})
                        except:
                            sigma_Q = {k:Q[k][j]}

                # Sum of utility function and sigma_Q
                try:
                    temp = copy.copy(f_table[j][1])
                    n_var=f_table[j][0]
                    for m in [index for index, value in sigma_Q.items()]:
                        # variable m's position in utility table
                        ind = [index for index, value in enumerate(n_var) if value == m]

                        #b = 2 ** (len(n_var) - (ind[0] + 1))

                        # 2^ (var m's position in utility table)
                        b = 2 ** (ind[0])
                        for l in range(len(temp)):
                            u=((math.floor(l / b))%2)
                            temp[l]=temp[l]+sigma_Q[m][1]*(1-u)
                        l=0
                        while l< (len(temp)-b):
                            temp[l]=temp[l+b]=max(temp[l],temp[l+b])
                            u = ((math.floor((l+1) / b)) % 2)
                            #l=l+2*u+1
                            l=l + (2 * u + 1)*(1-b%2) + (2 * u)*(b%2)
                    R[j][i] = [temp[0],temp[2**i]]
                except:
                    pass

            try:
                del sigma_Q
            except:
                pass
    return R

def Utulity_table(G):
      
    No_fac=len([n for n,attrdict in G.node.items() if attrdict['type'] == 'Factor' ])
    
    #implement some Utility function based on travel time and makespan and other parameters
    reward=10
    
    f_table={0: 0}
    for i in range(No_fac):
        a=len(G.neighbors('F_'+str(i)))
        f_table.update({i: [int(reward*(math.log(1+bin(j).count('1')) )) for j in range(2**a)]})        
    #f={0: [0, 2, 4, 6, 3, 5, 6, 7], 1: [0,4], 2: [0, 1, 2, 6]}
    #f_table={0:[[0],[0,10]],1:[[0,1,2],[0,2,4,15,5,12,10,20]],2:[[1],[0,8]],3:[[2],[0,14]]}
    #f_table={0:[[0,1],[0,5,2,10]],1:[[0],[0,6]],2:[[0],[2,8]],3:[[1],[0,4]]}

    return f_table

def Z_func(G,Z,R):
    for i in range(No_var):
        #for j in [index for index, value in enumerate(variables[i]) if value == 1]:
        q=map(int,re.findall('\d+', ''.join(G.neighbors('V_'+str(i)))))
        tmp=list(q)
        tmp.sort()
        for j in tmp:
            try:
                z.update({i: [sum(x) for x in zip(z[i],R[j][i])]})
            except:
                z= {i:R[j][i]}
                z[i].reverse()
        Z.update(z)
        del z
    return Z


'''
#Create Random Factor Graph
'''
#factors={0:[1,0,1,1],1:[1,0,0,0],2:[1,0,1,0]}
#variables={0:[1,1,1],1:[0,0,0],2:[1,0,1],3:[1,0,0]}

#factors={0:[1,0,0],1:[1,1,1],2:[0,1,0],3:[0,0,1]}
#variables={0:[1,1,0,0],1:[0,1,1,0],2:[0,1,0,1]}

G=nx.Graph()
'''
G.add_nodes_from(['F_0','F_1','F_2','F_3'],type='Factor')
G.add_nodes_from(['V_0','V_1','V_2'],type='Var')
G.add_edges_from([('F_0','V_0'),('F_1','V_0'),('F_1','V_1'),('F_1','V_2'),('F_2','V_1'),('F_3','V_2')])
'''
G.add_nodes_from(['F_0','F_1','F_2','F_3'],type='Factor')
G.add_nodes_from(['V_0','V_1'],type='Var')
G.add_edges_from([('F_0','V_0'),('F_0','V_1'),('F_1','V_0'),('F_2','V_0'),('F_3','V_1')])

No_fac=len([n for n,attrdict in G.node.items() if attrdict['type'] == 'Factor' ])
No_var=len([n for n,attrdict in G.node.items() if attrdict['type'] == 'Var' ])

#nx.draw(G,with_labels=True,node_size=1000)
#plt.show()

'''
Utility function 
'''
f_table=Utulity_table(G)
print('Utility_table: ',f_table)

'''
Max-Sum Algorithm
'''
#while True:

    # for each neighbor
    #for
R = {0: 0}
for i in range(No_fac):
    R.update({i: [[0,0] for j in range(No_var)]})
Q = {0: 0}
for i in range(No_var):
    Q.update({i: [[0,0] for j in range(No_fac)]})

#Start message passing process with leaf nodes

# Each variable leaf node
for i in range(No_var):
    #if len([index for index, value in enumerate(variables[i]) if value == 1])==1:
    if len(G.neighbors('V_'+str(i)))==1:
        #for j in [index for index, value in enumerate(variables[i]) if value == 1]:
        q=map(int,re.findall('\d+', ''.join(G.neighbors('V_'+str(i)))))
        tmp=list(q)
        tmp.sort()
        for j in tmp:
            Q[i][j][1] = 1
# Each function leaf node
for j in range(No_fac):
    if len(G.neighbors('F_'+str(j)))==1:
        q=map(int,re.findall('\d+', ''.join(G.neighbors('F_'+str(j)))))
        tmp=list(q)
        tmp.sort()
        for i in tmp:
            R[j][i]= copy.copy(f_table[j])
print("R:", R)

#Each time a node receives a message from an edge e, it computes outgoing messages

#Calculate Q message, from variable to function
Q=Qmessage(G,Q,R)

print("Q: ",Q)

#Calculate R message, from function to variable
R=Rmessage(G,Q,R,f_table)
print("R: ",R)

Z={0: 0}
Z=Z_func(G,Z,R)

print('Z: ',Z)

# divide the (utility value for one robot)

