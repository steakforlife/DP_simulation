#setting

import random

global seedNumber
seedNumber=10
def init():
    global GraphSize
    global NumberofObstacle
    global AdaptionSpeed
    global SINR_Constraint
    global D
    global Lambda
    global destination
    global UpperBoundMRC
    global AverageSickness
    global SicknessMax
    global TO_Max
    global RO_Max
    GraphSize=20
    destination=GraphSize-1
    NumberofObstacle=3
    AdaptionSpeed=1
    SINR_Constraint=20
    D=100
    Lambda=0
    UpperBoundMRC=2
    AverageSickness=30
    SicknessMax=222
    TO_Max=1.26
    RO_Max=1.24



def constructGraph(graph,M,Obstacles):
    #v1,v2,sickness,SINR,edgeLength,MRC 
    random.seed(seedNumber)
    graph.clear()
    Obstacles.clear()
    M.clear()
    #graph.append([0,1,random.randint(0,SicknessMax),random.randint(0,SINR_Constraint),random.randint(0,GraphSize),random.uniform(0,TO_Max+RO_Max)])
    graph.append([0,destination,100000,0,10000,random.uniform(0,TO_Max+RO_Max)])
    for i in range(destination):#0~18
        graph.append([i,i+1,random.randint(0,SicknessMax),random.randint(0,SINR_Constraint),random.randint(0,GraphSize),random.uniform(0,TO_Max+RO_Max)])
        graph.append([random.randint(0,destination),random.randint(0,destination),random.randint(0,SicknessMax),random.randint(0,SINR_Constraint),random.randint(0,GraphSize),random.uniform(0,TO_Max+RO_Max)])

    O=random.sample(range(1, GraphSize-1), NumberofObstacle)
    Obstacles[:]=list(O)
    #Obstacles+Obstacles
    d={}
    for i in range(GraphSize+1):
        a=()
        for j in range(GraphSize+1):
            if(j!=i):
               a=a+(random.randint(1,SINR_Constraint-1),) 
            else: 
                a=a+(SINR_Constraint,)
        d[str(i)]=a
    #d={'0': (10,2,3,4,1),'1': (2,10,6,5,4),'2': (1,4,10,6,4),'3': (2,2,4,10,3),'4': (1,3,3,5,10)}
    M.update(d)
    #print(graph, "\n")

def plot():
    global CostMy, QfuncMy, PathMy, MRC_My,SINR_My
    CostMy=[]
    QfuncMy=[]
    PathMy=[]
    MRC_My=[]
    SINR_My=[]
    #============
    global CostARC, QfuncARC, PathARC, MRC_ARC, SINR_ARC
    CostARC=[]
    QfuncARC=[]
    PathARC=[]
    MRC_ARC=[]
    SINR_ARC=[]
    #============
    global CostInR, QfuncInR,PathInR, MRC_InR, SINR_InR
    CostInR=[]
    QfuncInR=[]
    PathInR=[]
    MRC_InR=[]
    SINR_InR=[]
    #============
    global CostRobot, QfuncRobot, PathRobot, MRC_Robot, SINR_Robot
    CostRobot=[]
    QfuncRobot=[]
    PathRobot=[]
    MRC_Robot=[]
    SINR_Robot=[]



# def addEdgeSetting(graph,v1,v2,sickness,SINR,edgeLength,MRC):
#     graph.append([v1,v2,sickness,SINR,edgeLength,MRC])