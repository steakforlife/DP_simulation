
import random
import matplotlib.pyplot as plt
import setting
#from DP_fundamental import *
import DP_fundamental as my
#from ARC import *
import ARC
#from IRSandRW import *
import IRSandRW
#from RobotPathPlanning import *
import RobotPathPlanning as Robot
import pathlib
import numpy as np
import matplotlib.ticker as plticker
from datetime import datetime
import time

def ExecutionTime():
    global execTime
    global start_time
    start_time=time.time()

#=====================================unit test=======================================================
x=[]
setting.init()
setting.plot()

Hset={0: 'Number of Obstacle', 1: 'Size of Area', 2: 'MRC Upper Bound', 3: 'Adaption Speed', 4: 'IRS Search Time'}
Vset={0: 'Cost Function',1: 'Path Length', 2: 'Cybersickness', 3: 'RET Magnitude', 4: 'SINR'}
Vertical='Cost Function'
Horizontal='MRC Upper bound'
setting.seedNumber=2001
avgExecTime=0
count=0
ExecutionTime()
countIteration=0
#=====================change number and range of iteration==================
#for i in range(0,25):
for i in range(0,18):
    a = my.GraphMy(setting.GraphSize)
    b = ARC.GraphARC(setting.GraphSize)
    c = IRSandRW.GraphIRSandRW(setting.GraphSize)
    d = Robot.GraphRobot(setting.GraphSize)
    setting.constructGraph(a.graph,a.M,a.Obstacle)
    setting.constructGraph(b.graph,b.M,b.Obstacle)
    setting.constructGraph(c.graph,c.M,c.Obstacle)
    setting.constructGraph(d.graph,d.M,d.Obstacle)
    a.myAlgo(0,setting.destination)
    b.ARC(0,setting.destination)
    c.IRSandRW(0,setting.destination)
    d.RobotPathPlanning(0,setting.destination)
    #==============step 1. change x==================================
    x.append(setting.NumberofObstacle)
    #print(setting.NumberofObstacle)
    #setting.Lambda+=1
    setting.NumberofObstacle+=1
    #setting.GraphSize+=1
    # setting.destination+=1
    #setting.AdaptionSpeed+=1
    # setting.Lambda+=1
    # if(setting.NumberofObstacle==15):
    #     setting.NumberofObstacle=14

    avgExecTime+=time.time()-start_time
    countIteration+=1

    # x.append(setting.UpperBoundMRC)
    #setting.UpperBoundMRC+=0.3
    #=============step 2. change y axis==============================
    Data_1,=plt.plot(x, setting.CostMy, 'r-.',label='MyAlgo',marker='^',markevery=1)
    Data_2,=plt.plot(x, setting.CostARC, 'g--',label='ARC',marker='*',markevery=1)
    Data_3,=plt.plot(x, setting.CostInR, 'b-',label='IRSandRW',marker='1',markevery=1)
    Data_4,=plt.plot(x, setting.CostRobot, 'y:',label='RobotPath',marker='+',markevery=1)

    # Data_1,=plt.plot(x, setting.PathMy, 'r-.',label='MyAlgo',marker='^',markevery=5)
    # Data_2,=plt.plot(x, setting.PathARC, 'g--',label='ARC',marker='*',markevery=5)
    # Data_3,=plt.plot(x, setting.PathInR, 'b-',label='IRSandRW',marker='1',markevery=5)
    # Data_4,=plt.plot(x, setting.PathRobot, 'y:',label='RobotPath',marker='+',markevery=5)

    # Data_1,=plt.plot(x, setting.QfuncMy, 'r-.',label='MyAlgo',marker='^',markevery=5)
    # Data_2,=plt.plot(x, setting.QfuncARC, 'g--',label='ARC',marker='*',markevery=5)
    # Data_3,=plt.plot(x, setting.QfuncInR, 'b-',label='IRSandRW',marker='1',markevery=5)
    # Data_4,=plt.plot(x, setting.QfuncRobot, 'y:',label='RobotPath',marker='+',markevery=5)

    # Data_1,=plt.plot(x, setting.MRC_My, 'r-.',label='MyAlgo',marker='^',markevery=1)
    # Data_2,=plt.plot(x, setting.MRC_ARC, 'g--',label='ARC',marker='*',markevery=1)
    # Data_3,=plt.plot(x, setting.MRC_InR, 'b-',label='IRSandRW',marker='1',markevery=1)
    # Data_4,=plt.plot(x, setting.MRC_Robot, 'y:',label='RobotPath',marker='+',markevery=1)

    # Data_1,=plt.plot(x, setting.SINR_My, 'r-.',label='MyAlgo',marker='^',markevery=3)
    # Data_2,=plt.plot(x, setting.SINR_ARC, 'g--',label='ARC',marker='*',markevery=3)
    # Data_3,=plt.plot(x, setting.SINR_InR, 'b-',label='IRSandRW',marker='1',markevery=3)
    # Data_4,=plt.plot(x, setting.SINR_Robot, 'y:',label='RobotPath',marker='+',markevery=3)

    #=====avg. exec. time==========================

avgExecTime=avgExecTime/countIteration
#print(a.graph)
#plt.tick_params(axis='both', labelsize=12, color='green')
#print(x)
print("Average execution time(s): ",avgExecTime,"s")
plt.minorticks_on()
plt.tick_params(axis='x', which='minor', bottom=False)
#=================step 3. change axis label=========================
plt.xlabel(Hset[0])
plt.ylabel(Vset[0])
plt.legend(handles=[Data_1,Data_2,Data_3,Data_4],loc='upper center',bbox_to_anchor=(0.5,1.1),ncol=4)
x_ticks=np.arange(min(x),max(x)+1,1)
#x_ticks=(5,7,11,16,17,21,22)
# y_ticks=np.arange(  )
plt.xticks(x_ticks)


pathlib.Path('Results/Results_unitTest').mkdir(parents=True,exist_ok=True)
#================step 4. change file name===========================
#plt.savefig('Results/Results_unitTest/_.png', bbox_inches='tight')
plt.savefig('Results/Results_unitTest/'+str(Hset[0])+'_'+str(Vset[0])+datetime.today().strftime('%m%d')+'.png', bbox_inches='tight')
plt.show()
plt.close()

