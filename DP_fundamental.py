SINR_Constraint=5.0
D=100
import networkx as nx



class Graph:

    def __init__(self,vertices):
        self.V=vertices
        self.graph=[]
        self.path=[]
        

    #edge contains sickness,signal,length information 
    def addEdge(self,v1,v2,sickness,SINR,edgeLength):
        self.graph.append([v1,v2,sickness,SINR,edgeLength])

    #==================================main algorithm=====================================
    #def constructGraph(self,)
    #main function to find the path from src to dst
    #with minimum accumulated sickness
    def fundamentalAlgo(self,src,dst):
        #remove illegal edges based on constraints
        for v1,v2,sickness,SINR,edgeLength in self.graph:
            if SINR>SINR_Constraint:
                  #self.graph.sickness=float("Inf") ㄖ 
                  self.graph.remove([v1,v2,sickness,SINR,edgeLength])
                  self.graph.append([v1,v2,float("Inf"),SINR,edgeLength])
        #initialize
        Qfunction=[float("Inf")]*self.V
        Qfunction[src]=0
        Distance=D
        Parent={}
        #
        for _ in range(self.V-1):
            for v1,v2,sickness,SINR,edgeLength in self.graph:
                if Qfunction[v1] != float("Inf") and Qfunction[v1]+sickness<Qfunction[v2]:
                    if Distance-edgeLength>0:
                        Qfunction[v2]=Qfunction[v1]+sickness
                        Distance=Distance-edgeLength
                        Parent[v2]=v1
        self.printArr(Qfunction)
        self.printPath(Parent,src,dst)

    def printArr(self,dist):
        print("Vertex Distance from source")
        for i in range(self.V):
            print("{0}\t\t{1}".format(i,dist[i]))

    #backtrace the path*
    def printPath(self,parent,src,dst):
        pi=dst
        print("============================================================")
        print("the path* is: {0} ".format(dst),end='')
        while(pi!=src):
            print("{0} ".format(parent[pi]),end='')
            pi=parent[pi]
            

    #calcculate the distance with obstacles
    def distanceFunction():
        print("123 ")


#==================input===============================
g = Graph(5)
g.addEdge(0, 1, 1, 2, 5)
g.addEdge(0, 2, 2, 5, 5)
g.addEdge(1, 2, 3, 3, 5)
g.addEdge(1, 3, 2, 6, 5)
g.addEdge(1, 4, 2, 2, 5)
g.addEdge(2, 3, 3, 0.5, 5)
g.addEdge(3, 1, 1, 3, 5)
g.addEdge(4, 3, 3, 1, 5)

g.fundamentalAlgo(0,3)