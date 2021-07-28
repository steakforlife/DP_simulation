#SINR_mapping
import networkx as nx
import numpy as np
from numpy.core.numeric import NaN
import random
import setting
#Input parameter
# SINR_Constraint=8
# D=100
# Lambda=3
# destination=20
# AdaptionSpeed=1
# UpperBoundMRC=10
class GraphRobot:

    def __init__(self,vertices):
        self.V=vertices
        self.graph=[]
        self.path=[]
        self.M={}
        self.Obstacle=[]
        #self.obstacle(NumofObstacle)

    #remove the edge bump into obstacle
    def obstacleAvoidance(self,Obstacle):
        for i in Obstacle:
            self.graph=[[v1,v2,sickness,SINR,edgeLength,MRC] for v1,v2,sickness,SINR,edgeLength,MRC\
                in self.graph if(v1!=i and v2!=i)]

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
    def RobotPathPlanning(self,src,dst):
        #remove illegal edges based on constraints
        # for v1,v2,sickness,SINR,edgeLength,MRC in self.graph:
        #     if SINR>SINR_Constraint:
        #           #self.graph.sickness=float("Inf")
        #           self.graph.remove([v1,v2,sickness,SINR,edgeLength,MRC])
        #           self.graph.append([v1,v2,float("Inf"),SINR,edgeLength])
        #initialize
        Qfunction=[float("Inf")]*self.V
        Qfunction[src]=0
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
        rhoOld=0
        Parent={}
        Parent[src]=-1
        #
        self.obstacleAvoidance(self.Obstacle)
        self.upperbound(setting.UpperBoundMRC)
        #
        x=0
        count=0
        for _ in range(self.V-1):
            for v1,v2,sickness,SINR,edgeLength,MRC in self.graph:
                #from 0 to v1
                #for x in range(v1+1):
                    if(PathLength[v1]+edgeLength<PathLength[v2]):
                        
                    #if (costFunction[v1][x]+sickness+self.penalty(v2,x,SINR_Constraint)<costFunction[v2][x]):
                        costFunction[v2][x]=costFunction[v1][x]+sickness+self.penalty(v2,x,setting.SINR_Constraint)-setting.AdaptionSpeed
                        print("costFunction[{0}][{1}]= ".format(v2,x),costFunction[v2][x])
                        if(costFunction[v2][x]==min(costFunction[v2][:])):
                            Qfunction[v2]=Qfunction[v1]+sickness-setting.AdaptionSpeed
                            PathLength[v2]=PathLength[v1]+edgeLength
                            RETmagnitude[v2]=RETmagnitude[v1]+MRC
                            AccumulatedSINR[v2]=AccumulatedSINR[v1]+SINR
                            Parent[v2]=v1
                            count+=1
                     #updata IRS when IRS is available       
                    if(count==setting.Lambda):
                        self.SINR_mapping_Robot(v2)
                        AccumulatedSINR[v2]=AccumulatedSINR[v1]+self.M[str(v2)][v2]
                        x=v2
                        rho[v2]=v2
                        count=0
        #self.printArr(Qfunction)
        self.printPath(Parent,src,dst)
        print("costfunctionRobot: ",costFunction[dst][int(rho[dst])])
        print("Qfunction: ",Qfunction[dst])
        print("TotalPathlength: ",PathLength[dst])
        print("RET magnitude: ",RETmagnitude[dst])
        print("Accumulated SINR: ",AccumulatedSINR[dst])

        #======for plot==================================
        setting.CostRobot.append(costFunction[dst][int(rho[dst])])
        setting.QfuncRobot.append(Qfunction[dst])
        setting.PathRobot.append(PathLength[dst])
        setting.MRC_Robot.append(RETmagnitude[dst])
        setting.SINR_Robot.append(AccumulatedSINR[dst])
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
        while(pi!=src):
            print("{0} ".format(parent[pi]),end='')
            pi=parent[pi]
            count+=1
            if count>dst:
                print("no optimal path")        
            
    #SINR configuration
    def SINR_mapping_Robot(self,state):
        #Edge(self,v1,v2,sickness,SINR,edgeLength,MRC):
        #state==v2
        #g.M[str(state)][]
        #M={'0'#location: (10,2,3,4,5)#SINR values
        for edges in self.graph:
            v2=edges[1]
            edges[3]=self.M[str(state)][v2]


    #calculate SINR penalty
    def penalty(self,location,IRSnow,SINR_Constraint):
        SINR=self.M[str(IRSnow)][location]
        if(SINR-SINR_Constraint<0):
            return abs(SINR-SINR_Constraint)
        else:
            return 0


    
    


#==================input===============================
# GraphSize=5
# NumberofObstacle=0
# AdaptionSpeed=0
# Lambda=4
# g = GraphRobot(GraphSize)
# #v1,v2,sickness,SINR,edgeLength,MRC 
# g.addEdge(0, 1, 1, 2, 5, 4)
# g.addEdge(0, 2, 4, 5, 5, 2)
# g.addEdge(1, 2, 3, 3, 5, 3)
# g.addEdge(1, 3, 2, 6, 5, 4)
# g.addEdge(1, 4, 2, 2, 5, 5)
# g.addEdge(2, 3, 3, 0.5, 5, 6)
# #g.addEdge(3, 1, 1, 3, 5)
# g.addEdge(3, 4, 3, 1, 5, 8)
# g.addEdge(0, 4, 10, 8, 5, 0)
# g.M={'0': (10,2,3,4,5),'1': (2,10,6,5,4),'2': (1,4,10,6,4),'3': (2,2,4,10,3),'4': (1,3,3,5,10)}
# g.Obstacle=random.sample(range(1,GraphSize-1),NumberofObstacle)
#g.RobotPathPlanning(0,4)
test_rho=np.zeros(5)
test=np.array([[1.234, 2.345, 4.543],[0.34, 12.545, -4.543]])
test=np.zeros((5,5))
test[2]=[1.234, 2.345, 4.543,0,0]
test[3]=[0.34, 12.545, -4.543,0,0]
# test=2
# print(g.M[str(test)][v2])
#test SINR mapping
