import random
import math
import copy

'''
#INPUT: set of tasks(time window, location, id), set of robots(initial position, each robot's speed, STN),
INPUTS: No_var, No_fac
'''

def Qmessage(variables,factors,Q):
    for i in range(len(variables)):
    # Collect Rmessages from all neighboring functions, except the Qmessage receiver
        for j in range(len(factors)):
            if j in [index for index, value in enumerate(variables[i]) if value==1]:
                for k in [index for index, value in enumerate(variables[i]) if value==1]:
                    if k!=j:
                        w=[sum(x) for x in zip(Q[i][j], R[k][i])]
                        Q[i][j]=w
    return Q

def Rmessage(variables, factors,Q,R,f_table):
    for j in range(len(factors)):
        for i in range(len(variables)):
            if i in [index for index, value in enumerate(factors[j]) if value==1]:
                for k in [index for index, value in enumerate(factors[j]) if value==1]:
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


'''
#Create Random Factor Graph
'''
#factors={0:[1,0,1,1],1:[1,0,0,0],2:[1,0,1,0]}
#variables={0:[1,1,1],1:[0,0,0],2:[1,0,1],3:[1,0,0]}
factors={0:[1,0,0],1:[1,1,1],2:[0,1,0],3:[0,0,1]}
variables={0:[1,1,0,0],1:[0,1,1,0],2:[0,1,0,1]}
No_var=len(variables)
No_fac=len(factors)
'''
Utility function 
'''

# Initialize utility func.
f_table={0: 0}
for i in range(No_fac):
    a=len([index for index, value in enumerate(factors[i]) if value == 1])
    f_table.update({i: [0 for j in range(2**a)]})


#implement some Utility function based on travel time and makespan and other parameters

#f={0: [0, 2, 4, 6, 3, 5, 6, 7], 1: [0,4], 2: [0, 1, 2, 6]}
f_table={0:[[0],[0,10]],1:[[0,1,2],[0,2,4,15,5,12,10,20]],2:[[1],[0,8]],3:[[2],[0,14]]}

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
    if len([index for index, value in enumerate(variables[i]) if value == 1])==1:
        for j in [index for index, value in enumerate(variables[i]) if value == 1]:
            Q[i][j][1] = 1
# Each function leaf node
for j in range(No_fac):
    if len([index for index, value in enumerate(factors[j]) if value == 1])==1:
        for i in [index for index, value in enumerate(factors[j]) if value == 1]:
            R[j][i][1] = f_table[j][1][1]
print("R:", R)

#Each time a node receives a message from an edge e, it computes outgoing messages

#Calculate Q message, from variable to function
Q=Qmessage(variables,factors,Q)

print("Q: ",Q)

#Calculate R message, from function to variable
R=Rmessage(variables, factors,Q,R,f_table)
print("R: ",R)


for i in range(No_var):
    for j in [index for index, value in enumerate(variables[i]) if value == 1]:
        try:
            z[i].reverse()
            z.update({i: [sum(x) for x in zip(z[i],R[j][i])]})
            h=0
        except:
            z= {i:R[j][i]}



# divide the (utility value for one robot)

















