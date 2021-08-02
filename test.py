
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
#=====================================unit test=======================================================
x=[]
setting.init()
setting.plot()
Hset={0: 'Number of Obstacle', 1: 'Size of Area', 2: 'MRC Upper Bound', 3: 'Adaption Speed', 4: 'IRS Search Time'}
Vset={0: 'Cost Function',1: 'Path Length', 2: 'Cybersickness', 3: 'MRC', 4: 'SINR'}
Vertical='Cost Function'
Horizontal='MRC Upper bound'
setting.seedNumber=100
#=====================change number and range of iteration==================
#for i in range(0,25):
for i in range(20):
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
    x.append(setting.Lambda)
    #print(setting.NumberofObstacle)
    setting.Lambda+=1
    # if(setting.NumberofObstacle==15):
    #     setting.NumberofObstacle=14
    

    # x.append(setting.UpperBoundMRC)
    # setting.UpperBoundMRC+=0.1
    #=============step 2. change y axis==============================
    Data_1,=plt.plot(x, setting.CostMy, 'r-.',label='MyAlgo',marker='^',markevery=5)
    Data_2,=plt.plot(x, setting.CostARC, 'g--',label='ARC',marker='*',markevery=5)
    Data_3,=plt.plot(x, setting.CostInR, 'b-',label='IRSandRW',marker='1',markevery=5)
    Data_4,=plt.plot(x, setting.CostRobot, 'y:',label='RobotPath',marker='+',markevery=5)
    # Data_1,=plt.plot(x, setting.SINR_My, 'r-.',label='MyAlgo')
    # Data_2,=plt.plot(x, setting.SINR_ARC, 'g--',label='ARC')
    # Data_3,=plt.plot(x, setting.SINR_InR, 'b-',label='IRSandRW')
    # Data_4,=plt.plot(x, setting.SINR_Robot, 'y:',label='RobotPath')
#print(a.graph)
#plt.tick_params(axis='both', labelsize=12, color='green')
#print(x)
plt.minorticks_on()
plt.tick_params(axis='x', which='minor', bottom=False)
#=================step 3. change axis label=========================
plt.xlabel(Hset[4])
plt.ylabel(Vset[0])
plt.legend(handles=[Data_1,Data_2,Data_3,Data_4],loc='upper center',bbox_to_anchor=(0.5,1.1),ncol=4)
x_ticks=np.arange(min(x),max(x)+2,5)
#x_ticks=(5,7,11,16,17,21,22)
# y_ticks=np.arange(  )
plt.xticks(x_ticks)


pathlib.Path('Results/Results_unitTest').mkdir(parents=True,exist_ok=True)
#================step 4. change file name===========================
#plt.savefig('Results/Results_unitTest/_.png', bbox_inches='tight')
plt.savefig('Results/Results_unitTest/'+str(Hset[4])+'_'+str(Vset[0])+'.png', bbox_inches='tight')
plt.show()
plt.close()

