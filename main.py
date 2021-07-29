#main
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

setting.init()
setting.plot()


# random.seed(setting.seedNumber)
# a = my.GraphMy(setting.GraphSize)
# setting.constructGraph(a.graph,a.M,a.Obstacle)
# print(a.graph)
# a.myAlgo(0,setting.destination)
# print(a.graph)
# print(a.M)
# print(a.Obstacle)


#-------------------------------

x=[]
Vset={0: 'Cost Function',1: 'Path Length', 2: 'Cybersickness', 3: 'MRC', 4: 'SINR'}
Hset={0: 'Number of Obstacle', 1: 'Size of Area', 2: 'MRC Upper Bound', 3: 'Adaption Time', 4: 'IRS Search Time'}
Hvar={0: setting.NumberofObstacle, 1: setting.GraphSize, 2: setting.UpperBoundMRC, 3: setting.AdaptionSpeed, 4: setting.Lambda}
Vertical='Cost Function'
Horizontal='IRS Search Time'
for v in Vset:
    for h in Hset:
        setting.init()
        setting.plot()
        x=[]
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
            if(h==0):
                x.append(setting.NumberofObstacle)
                if(i%2==0):
                    setting.NumberofObstacle+=1
            elif(h==1):
                x.append(setting.GraphSize)
                setting.GraphSize+=1
                setting.destination+=1
            elif(h==2):
                x.append(setting.UpperBoundMRC)
                setting.UpperBoundMRC+=0.1
            elif(h==3):
                x.append(setting.AdaptionSpeed)
                setting.AdaptionSpeed+=1
            elif(h==4):
                x.append(setting.Lambda)
                setting.Lambda+=1

            #x=[1]
            if(v==0):
                Data_1,=plt.plot(x, setting.CostMy, 'r-.^',label='MyAlgo')
                Data_2,=plt.plot(x, setting.CostARC, 'g--*',label='ARC')
                Data_3,=plt.plot(x, setting.CostInR, 'b-v',label='IRSandRW')
                Data_4,=plt.plot(x, setting.CostRobot, 'y:+',label='RobotPath')
            elif(v==1):
                Data_1,=plt.plot(x, setting.PathMy, 'r-.^',label='MyAlgo')
                Data_2,=plt.plot(x, setting.PathARC, 'g--*',label='ARC')
                Data_3,=plt.plot(x, setting.PathInR, 'b-v',label='IRSandRW')
                Data_4,=plt.plot(x, setting.PathRobot, 'y:+',label='RobotPath')
            elif(v==2):
                Data_1,=plt.plot(x, setting.QfuncMy, 'r-.^',label='MyAlgo')
                Data_2,=plt.plot(x, setting.QfuncARC, 'g--*',label='ARC')
                Data_3,=plt.plot(x, setting.QfuncInR, 'b-v',label='IRSandRW')
                Data_4,=plt.plot(x, setting.QfuncRobot, 'y:+',label='RobotPath')
            elif(v==3):
                Data_1,=plt.plot(x, setting.MRC_My, 'r-.^',label='MyAlgo')
                Data_2,=plt.plot(x, setting.MRC_ARC, 'g--*',label='ARC')
                Data_3,=plt.plot(x, setting.MRC_InR, 'b-v',label='IRSandRW')
                Data_4,=plt.plot(x, setting.MRC_Robot, 'y:+',label='RobotPath')
            elif(v==4):
                Data_1,=plt.plot(x, setting.SINR_My, 'r-.^',label='MyAlgo')
                Data_2,=plt.plot(x, setting.SINR_ARC, 'g--*',label='ARC')
                Data_3,=plt.plot(x, setting.SINR_InR, 'b-v',label='IRSandRW')
                Data_4,=plt.plot(x, setting.SINR_Robot, 'y:+',label='RobotPath')

            #plt.plot(x,setting.CostMy,'r-.^',x, setting.CostARC, 'g--*',x, setting.CostInR, 'b-v', x, setting.CostRobot, 'y:+') #畫線
        plt.tick_params(axis='both', labelsize=12, color='green')
        plt.xlabel(Hset[h])
        plt.ylabel(Vset[v])
        plt.legend(handles=[Data_1,Data_2,Data_3,Data_4],loc='upper center',bbox_to_anchor=(0.5,1.1),ncol=4)
        plt.savefig('Results_{2}/{0}_{1}.png'.format(Hset[h],Vset[v],setting.seedNumber), bbox_inches='tight')
        plt.show(block=False)
        plt.pause(3)
        plt.close()





