#SINR_mapping
from pathlib import Path
import setting
from math import cos
import networkx as nx
import numpy as np
from numpy.core.numeric import Inf, NaN
import random
#Input parameter


class GraphMy:

    def __init__(self,vertices):
        self.V=vertices
        self.graph=[]
        self.path=[]
        self.M={}
        self.Obstacle=[]
        # self.SINR_Constraint=8
        # self.D=100
        # self.Lambda=3
        # self.destination=20
        # self.AdaptionSpeed=1
        # self.UpperBoundMRC=10
        #self.obstacle(NumofObstacle)

    #remove the edge bump into obstacle
    def obstacleAvoidance(self,Obstacle):
        for i in Obstacle:
            self.graph=[[v1,v2,sickness,SINR,edgeLength,MRC] for v1,v2,sickness,SINR,edgeLength,MRC in self.graph if(v1!=i and v2!=i)]

    #remove the edge which exceed MRC upper bound
    def upperbound(self,UpperBoundMRC):
        self.graph=[[v1,v2,sickness,SINR,edgeLength,MRC] for v1,v2,sickness,SINR,edgeLength,MRC\
            in self.graph if(MRC<UpperBoundMRC)]
          
    #edge contains sickness,signal,length,MRC information 
    def addEdge(self,v1,v2,sickness,SINR,edgeLength,MRC):
        self.graph.append([v1,v2,sickness,SINR,edgeLength,MRC])

    #rho for the different location, psi for the IRS element phase shift
    def addConfigurationSINR(self,rho,psi):
        self.M.append([rho,psi])

    #==================================main algorithm=====================================
    #def constructGraph(self,)
    #main function to find the path from src to dst
    #with minimum accumulated sickness
    def myAlgo(self,src,dst):
        #remove illegal edges based on constraints
        # for v1,v2,sickness,SINR,edgeLength,MRC in self.graph:
        #     if SINR>SINR_Constraint:
        #           #self.graph.sickness=float("Inf")
        #           self.graph.remove([v1,v2,sickness,SINR,edgeLength,MRC])
        #           self.graph.append([v1,v2,float("Inf"),SINR,edgeLength])
        #initialize
        Qfunction=np.zeros((self.V+1,self.V+1))
        Qfunction[:][:]=float("Inf")
        Qfunction[src][:]=0
        #
        costFunction=np.zeros((self.V+1,self.V+1))
        costFunction[:][:]=float("Inf")
        costFunction[src][:]=0
        #
        PathLength=[float("Inf")]*self.V
        PathLength[src]=0
        #
        RETmagnitude=[float("Inf")]*self.V
        RETmagnitude[src]=0
        #
        AccumulatedSINR=[float("Inf")]*self.V
        AccumulatedSINR[src]=0
        #
        #Distance=D
        rho=np.zeros(self.V)
        rhoLast=-1
        firstIRS=False
        Parent={}
        for i in range(self.V):
            Parent[i]=-1
        #
        self.obstacleAvoidance(self.Obstacle)
        self.upperbound(setting.UpperBoundMRC)
        #
        #edgeNumber=0
        #print("Lambda=",setting.Lambda)
        for _ in range(self.V-1):
            for v1,v2,sickness,SINR,edgeLength,MRC in self.graph:
                #edgeNumber+=1
                #from 0 to v1
                #for x in range(0,i):
                #x=v1
                #while (x!=-1):
                for x in range(v1+1):
                    #print(x)
                    #find min cost under no IRS adjustment
                    if (costFunction[v1][x]+(sickness)*setting.alleviation+self.penalty(v2,x)<costFunction[v2][x]):
                        # print("(v1,v2): ",v1,v2)
                        # print("x: ",x)
                        costFunction[v2][x]=costFunction[v1][x]+(sickness)*setting.alleviation+\
                        self.penalty(v2,x)-setting.AdaptionSpeed
                        #print("costfunction[{0}][{1}]= ".format(v2,x),costFunction[v2][x])
                        rho[v2]=x
                        #if(costFunction[v2][x]==min(costFunction[v2][:])):
                        Parent[v2]=v1
                        Qfunction[v2]=Qfunction[v1]+(sickness)*setting.alleviation-setting.AdaptionSpeed
                        PathLength[v2]=PathLength[v1]+edgeLength
                        RETmagnitude[v2]=RETmagnitude[v1]+MRC*setting.alleviation
                        AccumulatedSINR[v2]=AccumulatedSINR[v1]+SINR
                    #x=Parent[x]
                    #print("v1,x= ",v1,x)
                #from 0 to j-lambda     
                
                #print("Parent[{0}],v2=".format(v2), Parent[v2])
                #if(firstIRS==False):
                if(v1==0):
                    #lastAdjust=Parent[v2]
                    lastAdjust=0
                    while(lastAdjust!=-1):
                        old=costFunction[v2][v2]
                        costFunction[v2][v2]=self.costFunctionUpdateWithIRS(costFunction[v1][0],sickness,costFunction[v2][v2])                  
                        #print("cost[v2][v2]",costFunction[v2][v2],v2)
                        #print("cost[v2][0]",costFunction[v2][0])

                        if(self.newIsSmaller(old,costFunction[v2][v2])):
                            #rho[v2]=v2
                            Parent[v2]=v1
                            Qfunction[v2]=Qfunction[v1]+(sickness)*setting.alleviation-setting.AdaptionSpeed
                            PathLength[v2]=PathLength[v1]+edgeLength
                            RETmagnitude[v2]=RETmagnitude[v1]+MRC
                        lastAdjust=Parent[lastAdjust]
                        #print("lastAdjust= ",lastAdjust)
                # if(v2-setting.Lambda<0):
                #     for i in range(v2):
                #         if(costFunction[v1][i]+sickness<costFunction[v2][v2]):
                #             #print("(v1,i): ",v1,i)
                #             costFunction[v2][v2]=costFunction[v1][i]+sickness-setting.AdaptionSpeed
                #             if(costFunction[v2][v2]==min(costFunction[v2][:])):
                #                 Parent[v2]=v1
                #                 Qfunction[v2]=Qfunction[v1]+sickness-setting.AdaptionSpeed
                #                 PathLength[v2]=PathLength[v1]+edgeLength
                #                 RETmagnitude[v2]=RETmagnitude[v1]+MRC
                            #print("costfunction[{0}][{1}]: ".format(v2,v2),costFunction[v2][v2])
                else:
                    
                    lastAdjust=Parent[v2]
                    #print("Lambda=",setting.Lambda)
                    for _ in range(setting.Lambda-1):
                        if(lastAdjust==-1):
                            #print("lastdjust=-1,v2=",v2)
                            break
                        lastAdjust=Parent[lastAdjust]
                        #print("lastAdjust=",lastAdjust)
                        if (lastAdjust==src):
                            #print("lastAdjust==src,v2=",v2)
                            break
                        
                    #print("(v2,lastAdjust)= ({0},{1})".format(v2,lastAdjust))
                    #print("lastAdjust= ",lastAdjust)
                    #how to represent gap with lambda
                    #while(lastAdjust!=-1 and lastAdjust!=0 ):
                    while(lastAdjust!=-1):
                    #for i in range(1+v1+1-setting.Lambda):
                        old=costFunction[v2][v2]
                        costFunction[v2][v2]=self.costFunctionUpdateWithIRS(costFunction[v1][lastAdjust],sickness,costFunction[v2][v2])
                        
                        if(self.newIsSmaller):
                            rho[v2]=v2
                            #Parent[v2]=v1
                            Qfunction[v2]=Qfunction[v1]+(sickness)*setting.alleviation-setting.AdaptionSpeed
                            PathLength[v2]=PathLength[v1]+edgeLength
                            RETmagnitude[v2]=RETmagnitude[v1]+MRC
                        lastAdjust=Parent[lastAdjust]
                        # else:
                        #     if(costFunction[v1][lastAdjust]!=Inf and costFunction[v2][lastAdjust]>costFunction[v1][lastAdjust]+(sickness)*setting.alleviation+\
                        #         self.penalty(v2,lastAdjust)-setting.AdaptionSpeed):
                        #         costFunction[v2][lastAdjust]=costFunction[v1][lastAdjust]+(sickness)*setting.alleviation+\
                        #         self.penalty(v2,lastAdjust)-setting.AdaptionSpeed
                        #     lastAdjust=Parent[lastAdjust]
                            
                    #lastAdjust=v2
                        #print("lastAdjust= ",lastAdjust)
                            #print("costfunction[{0}][{1}]: ".format(v2,v2),costFunction[v2][v2])
                
                # rho[v2]=np.argmin(costFunction[v2],axis=0)
                if(rho[v2]==v2):
                    #print("v2=",v2)
                    #print("rho[{0}]= ".format(v2),rho[v2])
                    #print("rhoLast= ",rhoLast)
                    #=========================update IRS==============================================
                    self.SINR_mapping_my(int(rho[v2]))
                    #costFunction[v2][int(rho[v2])]-=self.penalty(v2,int(rho[v2]),setting.SINR_Constraint)
                    AccumulatedSINR[v2]=AccumulatedSINR[v1]+self.M[str(v2)][v2]
                #rho[v2]=np.argmin(costFunction[v2],axis=0)
                    #print(rhoLast)
                #costFunction[v2][int(rho[v2])]=min(costFunction[v2])

        rho[v2]=np.argmin(costFunction[v2],axis=0)
        #self.printArr(Qfunction)
        self.printPath(Parent,src,dst)
        print("costfunction[{0}][{1}]: ".format(dst,int(rho[dst])),costFunction[dst][int(rho[dst])])
        #print(costFunction[dst][:])
        #print(self.M)
        # # for i in range(dst+1):
        # #     print(rho[i])
        # # for i in range(dst+1):
        # #     print("Parent[{0}]= {1}".format(i,Parent[i]))
        #print("Qfunction: ",Qfunction[dst][int(rho[dst])])
        #print("TotalPathlength: ",PathLength[dst])
        #print("RET magnitude: ",RETmagnitude[dst])
        #print("Accumulated SINR: ",AccumulatedSINR[dst])

        #======for plot==================================
        setting.CostMy.append(costFunction[dst][int(rho[dst])])
        setting.QfuncMy.append(Qfunction[dst][int(rho[dst])])
        setting.PathMy.append(PathLength[dst])
        setting.MRC_My.append(RETmagnitude[dst])
        setting.SINR_My.append(AccumulatedSINR[dst])
        #================================================

    def printArr(self,dist):
        print("Vertex Distance from source")
        for i in range(self.V):
            print("{0}\t\t{1}".format(i,dist[i]))

    #backtrace the path*
    def printPath(self,parent,src,dst):
        pi=dst
        count=0
        print("============================================================")
        print("the path* is: {0} ".format(pi),end='')
        #print(parent[pi])
        while(pi!=src):
            if(parent[pi]==-1):
                print("no optimal path")
                return
            print("{0} ".format(parent[pi]),end='')
            pi=parent[pi]
            count+=1
            if count>dst:
                print("no optimal path")   
        print("\n")     
            
    #SINR configuration
    def SINR_mapping_my(self,state):
        #Edge(self,v1,v2,sickness,SINR,edgeLength,MRC):
        #state==v2
        #g.M[str(state)][]
        #M={'0'#location: (10,2,3,4,5)#SINR values
        for edges in self.graph:
            v2=edges[1]
            edges[3]=self.M[str(state)][v2]


    #calculate SINR penalty
    def penalty(self,location,IRSnow):
        SINR=self.M[str(IRSnow)][location]
        if(SINR-setting.SINR_Constraint<0):
            #print(abs(SINR-setting.SINR_Constraint))
            return abs(SINR-setting.SINR_Constraint)     
        else:
            return 0

    def costFunctionUpdateWithIRS(self,costFunctionFrom,sickness,costFunctionTo):
        if(costFunctionFrom+sickness*setting.alleviation<costFunctionTo):
            costFunctionTo=costFunctionFrom+sickness*setting.alleviation-setting.AdaptionSpeed
        return costFunctionTo
        
    def newIsSmaller(self,old,new):
        if(old>new):
            return True
        else:
            return False





#==================input===============================
# GraphSize=5
# NumberofObstacle=0
# AdaptionSpeed=0
# Lambda=2
g = GraphMy(5)
#v1,v2,sickness,SINR,edgeLength,MRC 
# g.addEdge(0, 1, 1, 8, 1, 4)
# g.addEdge(0, 2, 4, 5, 3, 2)
# g.addEdge(1, 2, 1, 8, 2, 3)
# g.addEdge(1, 3, 2, 6, 4, 4)
# g.addEdge(1, 4, 2, 2, 3, 5)
# g.addEdge(2, 3, 3, 8, 5, 6)
# g.addEdge(3, 1, 1, 3, 5, 0)
# g.addEdge(3, 4, 3, 8, 5, 8)
# g.addEdge(0, 4, 10, 1, 5, 0)
#===================================
g.addEdge(0, 1, 1, 2, 5, 4)
g.addEdge(0, 2, 4, 5, 5, 2)
g.addEdge(1, 2, 3, 3, 5, 3)
g.addEdge(1, 3, 2, 6, 5, 4)
g.addEdge(1, 4, 2, 2, 5, 5)
g.addEdge(2, 3, 3, 0.5, 5, 6)
#g.addEdge(3, 1, 1, 3, 5)
g.addEdge(3, 4, 3, 1, 5, 8)
g.addEdge(0, 4, 10, 8, 5, 0)
g.M={'0': (10,2,3,4,1),'1': (2,10,6,5,4),'2': (1,4,10,6,4),'3': (2,2,4,10,3),'4': (1,3,3,5,10)}
#g.Obstacle=random.sample(range(1,GraphSize-1),NumberofObstacle)
#g.myAlgo(0,4)
print()
test_rho=np.zeros(5)
test=np.array([[1.234, 2.345, 4.543],[0.34, 12.545, -4.543]])
test=np.zeros((5,5))
test[2]=[1.234, 2.345, 4.543,0,0]
test[3]=[0.34, 12.545, -4.543,0,0]
# test=2
# print(g.M[str(test)][v2])
#test SINR mapping
