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
a = my.GraphMy(setting.GraphSize)
b = ARC.GraphARC(setting.GraphSize)
c = IRSandRW.GraphIRSandRW(setting.GraphSize)
d = Robot.GraphRobot(setting.GraphSize)
#setting.AdaptionSpeed+=0.1

random.seed(10)


#-------------------------------
setting.constructGraph(a.graph,a.M,a.Obstacle)
# print(a.graph)
# print(a.M)
# print(a.Obstacle)
setting.constructGraph(b.graph,b.M,b.Obstacle)
setting.constructGraph(c.graph,c.M,c.Obstacle)
setting.constructGraph(d.graph,d.M,d.Obstacle)
x=[]
Vertical='Cost'
Horizontal='AdaptionSpeed'
for i in range(5):
    a.myAlgo(0,setting.destination)
    b.ARC(0,setting.destination)
    c.IRSandRW(0,setting.destination)
    d.RobotPathPlanning(0,setting.destination)
    x.append(i)
    setting.AdaptionSpeed+=1
# a.myAlgo(0,setting.destination)
# b.ARC(0,setting.destination)
# c.IRSandRW(0,setting.destination)
# d.RobotPathPlanning(0,setting.destination)

#x=[1]
Data_1,=plt.plot(x, setting.CostMy, 'r-.^',label='MyAlgo')
Data_2,=plt.plot(x, setting.CostARC, 'g--*',label='ARC')
Data_3,=plt.plot(x, setting.CostInR, 'b-v',label='IRSandRW')
Data_4,=plt.plot(x, setting.CostRobot, 'y:+',label='RobotPath')

#plt.plot(x,setting.CostMy,'r-.^',x, setting.CostARC, 'g--*',x, setting.CostInR, 'b-v', x, setting.CostRobot, 'y:+') #畫線
plt.tick_params(axis='both', labelsize=24, color='green')
plt.legend(handles=[Data_1,Data_2,Data_3,Data_4])
plt.savefig('{0}_{1}.png'.format(Horizontal,Vertical), bbox_inches='tight')
plt.show()





