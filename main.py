__author__ = 'caner.hekimoglu'
from numpy import *
import matrices
import dijkstra
import bfs
import astar
import gmplot

class strctCity:#city struct for bellmanFord
    def __init__(self):
        self.frm = 0
        self.dist = 0

    def __str__(self): #toString
        return str(self.frm) + " " + str(self.dist)


def findConnections(a, mat):#returns a list of city connected to city a in matrix without graph
    i = a + 1
    conList = []
    while i < (mat.shape[0]):
        if (mat[a][i] > 0):
            conList.append(i)
        i = i + 1
    i = 0
    while i < a:
        if (mat[i][a] > 0):
            conList.append(i)
        i = i + 1
    return conList


def nearesneighbour(mat,stName,enName):
    start = getCityIndex(stName)
    end = getCityIndex(enName)
    retStr=""
    bCondition=True
    startSet = set()
    startSet.add(start)
    current=start
    totalcost=0
    next=-1
    way=[]
    way.append(current)
    while(bCondition):
        bCondition=False
        checkList = findConnections(current, mat)
        min=99999
        if(current!=end):
            for i in range(0, len(checkList)):
                if(checkList[i] not in startSet):
                    newDist=mat[current][checkList[i]]
                    if(newDist==0):
                        newDist = mat[checkList[i]][current]
                    if(min>newDist):
                        min=newDist
                        next=checkList[i]
        if(min!=99999):
            current=next
            way.append(current)
            startSet.add(current)
            bCondition=True
            totalcost=totalcost+min
        else:
            j=0
            length=len(way)
            while j < length:
                retStr= retStr+ cities[way[j]] + ", "
                j = j + 1
            if(current==end):
                return ("Nearest Neighbour between "+stName +" to "+ enName+" path: "+retStr +" Cost "+ str(totalcost)),way
            else:
                return ("No way between "+stName +" to "+ enName+" path: "+retStr + "Current Cost "+ str(totalcost)),way


def formatroute(l, s, e):#returns bellmanford path as a list
    i = e
    cities = [e]
    while i != s:
        cities.append(l[i].frm)
        i = l[i].frm
    return cities


def hillclimb(dismat,airmatr,stName,enName):
    start = getCityIndex(stName)
    end = getCityIndex(enName)
    retStr=""
    bCondition=True
    startSet = set()
    startSet.add(start)
    current=start
    totalcost=0
    next=-1
    way=[]
    way.append(current)
    while(bCondition):
        bCondition=False
        checkList = findConnections(current, dismat)
        min=99999
        minDist=0
        if(current!=end):
            for i in range(0, len(checkList)):
                if(checkList[i] not in startSet):
                    newDist=dismat[current][checkList[i]]
                    if(newDist==0):
                        newDist = dismat[checkList[i]][current]
                    aird=airmatr[end][checkList[i]]
                    if(min>aird):
                        min=aird
                        next=checkList[i]
                        minDist=newDist
        if(min!=99999):
            current=next
            way.append(current)
            startSet.add(current)
            bCondition=True
            totalcost=totalcost+minDist
        else:
            j=0
            length=len(way)
            while j < length:
                retStr= retStr+ cities[way[j]] + ", "
                j = j + 1
            if(current==end):
                return ("HillClimb: between "+stName +" to "+ enName+" path: "+retStr +" Cost "+ str(totalcost)),way
            else:
                return ("Hill Climb: No way between "+stName +" to "+ enName+" path: "+retStr + "Current Cost "+ str(totalcost)),way


def printastar(lc):#prints results of bellman ford algorithm
    i = 0
    l = lc[0]
    cost = lc[1]
    l.reverse()
    length = len(l)
    stri = "Astar Algorithm from: " + cities[l[0]] + " to: " + cities[l[(length - 1)]] + ", Path : "
    while i < length:
        stri = stri + cities[l[i]] + ", "
        i = i + 1
    print(stri + "; Cost:" + str(cost))
    return stri+ "; Cost:" + str(cost)



def impl(conMat, distMat, stName, enName):#Bellman-Ford Algorithm Implementation
    change = 1
    t = 0
    l = []
    start = getCityIndex(stName)
    end = getCityIndex(enName)
    while t < conMat.shape[0]:
        s = strctCity()
        s.frm = -1
        s.dist = None
        l.append(s)
        t = t + 1
    startSet = set()
    startSet.add(start)
    while (change != 0):
        change = 0
        checkList = []
        for i in range(0, len(startSet)):
            startList = list(set(startSet))
            checkList = findConnections(startList[i], distMat)
            for j in range(0, len(checkList)):
                newDist=distMat[startList[i]][checkList[j]]
                if(newDist==0):
                    newDist = distMat[checkList[j]][startList[i]]
                distance =  newDist
                extra = 0
                if (l[startList[i]].dist != None):
                    extra = l[startList[i]].dist
                if ((l[checkList[j]].dist == None) or (l[checkList[j]].dist > (distance + extra))):
                    l[checkList[j]].dist = distance + extra
                    l[checkList[j]].frm = startList[i]
                    change = 1
                """k=0
                stri=""
                while k<l.__len__():
                    stri= stri+ str(l[k])+" ,"
                    k=k+1
                print("i:"+str(startList[i])+" j:"+str(checkList[j]))
                print(stri+"\n")"""
            startSet.update(checkList)
    """k=0
    stri=""
    while k<l.__len__():
        stri= stri+ str(l[k])+" ,"
        k=k+1
    print(stri+"\n")"""
    return formatroute(l, start, end), l[end].dist

def printbFord(lc):#prints results of bellman ford algorithm
    i = 0
    l = lc[0]
    cost = lc[1]
    l.reverse()
    length = len(l)
    stri = "Bellman Ford- from: " + cities[l[0]] + " to: " + cities[l[(length - 1)]] + ", Path : "
    while i < length:
        stri = stri + cities[l[i]] + ", "
        i = i + 1
    print(stri + "; Cost:" + str(cost))
    return stri+ "; Cost:" + str(cost)

def printDijkstra(mat,fromcity,tocity):
    fro=getCityIndex(fromcity)
    toc=getCityIndex(tocity)
    durgraph=makegraph(mat)
    cost,path=dijkstra.shortest_path(durgraph,fro,toc)
    length=len(path)
    stri= "Dijkstra- from: " + cities[path[0]] + " to: " + cities[path[(length - 1)]] + ", Path : "
    i=0
    while i < length:
        stri = stri + cities[path[i]] + ", "
        i = i + 1
    print(stri + "; Cost:" + str(cost))
    return (stri+ "; Cost:" + str(cost)),path

def getCityIndex(name):  # returns city name wrt index
    return cities.index(name)


def reshapematrix(x, a, b):  # reshape data to axb matrice
    return reshape(x, (a, b))


def floydWarshall(conMat, distMat, stName, enName):#Floyd-Warshall Algorithm Implementation
    dist = []
    length = conMat.shape[0]
    parent = []
    start = getCityIndex(stName)
    end = getCityIndex(enName)

    # infinity
    for i in range(0, length):
        dist.append([])
        parent.append([])
        for j in range(0, length):
            dist[i].append(99999)
            parent[i].append(0)

    # fill distance
    for i in range(0, length):
        for j in range(0, length):
            if (distMat[i][j] == 0):
                if (dist[i][j] != 99999):
                    dist[i][j] = 99999
            else:
                dist[i][j] = distMat[i][j]
                dist[j][i] = distMat[i][j]

    # initialize the path matrix
    for i in range(0, length):
        for j in range(0, length):
            if dist[i][j] == 99999:
                parent[i][j] = 0
            else:
                parent[i][j] = i

    for k in range(0, length):
        for i in range(0, length):
            for j in range(0, length):
                if dist[i][j] > dist[i][k] + dist[k][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    parent[i][j] = parent[k][j]

    print("FloydWarshall- From :" + stName + " To :" + enName + " Path : " + stName + ", " + pathRecursive(start, end, dist,
                                                                                            parent) + enName + ";" + "Cost:",
          dist[start][end])
    return "FloydWarshall- From :" + stName + " To :" + enName + " Path : " + stName + ", " + pathRecursive(start,end,dist,parent)+enName +";"+"Cost:"+str(dist[start][end])


def pathRecursive(i, j, dista, parenta):#recursive path finder from parent matrice
    if dista[i][j] == 99999:
        return " no path to "
    if parenta[i][j] == i:
        return " "
    else:
        return pathRecursive(i, parenta[i][j], dista, parenta) + str(cities[parenta[i][j]]) + ", " + pathRecursive(
            parenta[i][j], j, dista, parenta)

def printBfs(matbfs,fromcity,tocity,airM):
    fro=getCityIndex(fromcity)
    toc=getCityIndex(tocity)
    pathbfs,goal=bfs.greedyBFS(makegraph(matbfs),fro,toc,airM)
    j=0
    length=len(pathbfs)
    retStr=""
    cost=0
    k=length-1
    pathnew=[]
    #find path
    while k>0:
        if(matbfs[fro][pathbfs[k]]>0):
            pathnew=pathbfs[k:]
            pathnew.insert(0,fro)
            break
        k=k-1
    #construct string
    while j < len(pathnew):
        retStr= retStr+ cities[pathnew[j]] + ", "
        j = j + 1
        if(j!=len(pathnew)):
            cost=cost+matbfs[pathnew[j-1]][pathnew[j]]
    if(goal==1):
        return ("Best First: between "+fromcity +" to "+ tocity+" path: "+retStr +" Cost "+ str(cost)),pathnew
    else:
        return ("No way with Best First: between "+fromcity +" to "+ tocity+" path: "+retStr +" Cost "+ str(cost)),pathnew

def a_star(matast,fromcity,tocity,airM):
    fro=getCityIndex(fromcity)
    toc=getCityIndex(tocity)
    pathast,goal=astar.Astar(makegraph(matast),fro,toc,airMat,matast)
    j=0
    length=len(pathast)
    retStr=""
    cost=0
    k=length-1
    pathnew=[]
    #find path
    while k>0:
        if(matast[fro][pathast[k]]>0):
            pathnew=pathast[k:]
            pathnew.insert(0,fro)
            break
        k=k-1
    #construct string
    while j < len(pathnew):
        retStr= retStr+ cities[pathnew[j]] + ", "
        j = j + 1
        if(j!=len(pathnew)):
            cost=cost+matast[pathnew[j-1]][pathnew[j]]
    if(goal==1):
        return "Astar : between "+fromcity +" to "+ tocity+" path: "+retStr +" Cost "+ str(cost)
    else:
        return "No way with Astar: between "+fromcity +" to "+ tocity+" path: "+retStr +" Cost "+ str(cost)

def printiterdeep(itermat,fromcity,tocity):
    g=makegraph(itermat)
    fro=getCityIndex(fromcity)
    toc=getCityIndex(tocity)
    a1,a2=g.iterdeep(fro, toc, 99)
    retStr=""
    j=0
    cost=0
    while j < len(a2):
        retStr= retStr+ cities[a2[j]] + ", "
        j = j + 1
        if(j!=len(a2)):
            cost=cost+itermat[a2[j-1]][a2[j]]
    if(a1):
        return ("Iterative Deep : between "+fromcity +" to "+ tocity+" path: "+retStr +" Cost "+ str(cost)),a2
    else:
        return ("No way with Iterative Deep: between "+fromcity +" to "+ tocity+" path: "+retStr +" Cost "+ str(cost)),a2



def makegraph(mat):
    Gr=dijkstra.Graph()
    for k in range(0,mat.shape[0]):
        Gr.add_node(k)
    for i in range(0, mat.shape[0]):
        le=findConnections(i,mat)
        for j in range(0, le.__len__()):
            Gr.add_edge(i,le[j],mat[i][le[j]])
    return Gr

def retFullMatrix(choice):
    retMat=None
    if(choice==0):
        retMat=reshapematrix(matrices.driveMat, citysize, citysize)
    elif(choice==1):
        retMat=reshapematrix(matrices.durMat, citysize, citysize)
    elif(choice==2):
        retMat=reshapematrix(matrices.airMat, citysize, citysize)
    for i in range(0, retMat.shape[0]):
        for j in range(0, retMat.shape[0]):
            retMat[j][i] = retMat[i][j]
    return retMat

def routerFunc(algo,matType,fromcity,tocity,opt,excity):
    strar="Error Wrong Parameters"
    savelist=[]
    if(opt==0):
        if(algo==1):
            if(matType==0):
                lc=impl(conMat, driveMat, fromcity, tocity)
                strar=printbFord(lc)
                savelist=lc[0]
            elif(matType==1):
                lc=impl(conMat, durMat, fromcity, tocity)
                strar=printbFord(lc)
                savelist=lc[0]
        elif(algo==0):
            if(matType==0):
                strar=floydWarshall(conMat, driveMatFlo, fromcity, tocity)
            elif(matType==1):
                strar=floydWarshall(conMat, durMatFlo, fromcity, tocity)
        elif(algo==2):
            if(matType==0):
                strar,savelist=printDijkstra(driveMatDij, fromcity, tocity)
            elif(matType==1):
                strar,savelist=printDijkstra(durMatDij, fromcity, tocity)
        elif(algo==3):
            if(matType==0):
                strar,savelist=nearesneighbour(driveMat,fromcity,tocity)
            elif(matType==1):
                strar,savelist=nearesneighbour(durMat,fromcity,tocity)
        elif(algo==4):
            if(matType==0):
                strar,savelist=hillclimb(driveMat,airMat,fromcity,tocity)
            elif(matType==1):
                strar,savelist=hillclimb(durMat,airMat,fromcity,tocity)
        elif(algo==5):
            if(matType==0):
                strar,savelist=printBfs(driveMatDij,fromcity,tocity,airMat)
            elif(matType==1):
                strar,savelist=printBfs(durMatDij,fromcity,tocity,airMat)
        elif(algo==6):
            if(matType==0):
                lc=impl(conMat, driveMat, fromcity, tocity)
                strar=printastar(lc)
                savelist=lc[0]
            elif(matType==1):
                lc=impl(conMat, durMat, fromcity, tocity)
                strar=printastar(lc)
                savelist=lc[0]
        elif(algo==7):
            if(matType==0):
                strar,savelist=printiterdeep(driveMatDij,fromcity, tocity)
            elif(matType==1):
                strar,savelist=printiterdeep(durMatDij,fromcity,tocity)

    elif(opt==1):
        tempmat=retFullMatrix(matType)
        tairmat=retFullMatrix(2)
        cindex=getCityIndex(excity)
        for i in range(0, tempmat.shape[0]):
            tempmat[i][cindex]=0
            tempmat[cindex][i]=0
            tairmat[i][cindex]=0
            tairmat[cindex][i]=0
        if(algo==1):
            lc=impl(conMat, tempmat, fromcity, tocity)
            strar=printbFord(lc)
            savelist=lc[0]
        elif(algo==0):
            strar=floydWarshall(conMat, tempmat, fromcity, tocity)
        elif(algo==2):
            strar,savelist=printDijkstra(tempmat, fromcity, tocity)
        elif(algo==3):
            strar,savelist=nearesneighbour(tempmat,fromcity,tocity)
        elif(algo==4):
            strar,savelist=hillclimb(tempmat,tairmat,fromcity,tocity)
        elif(algo==5):
            strar,savelist=printBfs(tempmat,fromcity,tocity,airMat)
        elif(algo==6):
            lc=impl(conMat, tempmat, fromcity, tocity)
            strar=printastar(lc)
            savelist=lc[0]
        elif(algo==7):
            strar,savelist=printiterdeep(tempmat,fromcity,tocity)
        strar="NEW route Without city "+excity+" "+strar
    gmap = gmplot.GoogleMapPlotter(38.41273, 27.13838, 5)

    l1=[]
    mi=0
    while(mi<len(savelist)):
        l1.append((matrices.latcities[savelist[mi]],matrices.longcities[savelist[mi]]))
        mi=mi+1
    if(len(l1)>0):
        a,b = zip(*l1)
        gmap.plot(a, b, 'cornflowerblue', edge_width=10)
    # Draw
    gmap.draw("result_map.html")
    return strar

cities = matrices.cities
citysize=len(cities)
conMat = reshapematrix(matrices.conMat, citysize, citysize)
driveMat = reshapematrix(matrices.driveMat, citysize, citysize)
durMat = reshapematrix(matrices.durMat, citysize, citysize)
airMat = reshapematrix(matrices.airMat, citysize, citysize)
driveMatFlo = reshapematrix(matrices.driveMat, citysize, citysize)
durMatFlo = reshapematrix(matrices.durMat, citysize, citysize)
for i in range(0, driveMatFlo.shape[0]):
    for j in range(0, driveMatFlo.shape[0]):
        driveMatFlo[j][i] = driveMatFlo[i][j]
for i in range(0, durMatFlo.shape[0]):
    for j in range(0, durMatFlo.shape[0]):
        durMatFlo[j][i] = durMatFlo[i][j]
driveMatDij = reshapematrix(matrices.driveMat, citysize, citysize)
durMatDij = reshapematrix(matrices.durMat, citysize, citysize)
for i in range(0, driveMatDij.shape[0]):
    for j in range(0, driveMatDij.shape[0]):
        driveMatDij[j][i] = driveMatDij[i][j]
for i in range(0, durMatDij.shape[0]):
    for j in range(0, durMatDij.shape[0]):
        durMatDij[j][i] = durMatDij[i][j]
if __name__ == "__main__":
    # init

    print("BellMan-Ford Drive Distance (km)\n")
    printbFord(impl(conMat, driveMat, "Izmir", "Sofia"))
    printbFord(impl(conMat, driveMat, "Izmir", "Athens"))
    printbFord(impl(conMat, driveMat, "Ankara", "Athens"))
    printbFord(impl(conMat, driveMat, "Antalya", "Skopje"))
    
    print("\nBellMan-Ford Duration (minutes)\n")
    printbFord(impl(conMat, durMat, "Izmir", "Sofia"))
    printbFord(impl(conMat, durMat, "Izmir", "Athens"))
    printbFord(impl(conMat, durMat, "Ankara", "Athens"))
    printbFord(impl(conMat, durMat, "Antalya", "Skopje"))

    print("\nFloyd Warshall drive (km)\n")
    floydWarshall(conMat, driveMatFlo, "Izmir", "Sofia")
    floydWarshall(conMat, driveMatFlo, "Izmir", "Athens")
    floydWarshall(conMat, driveMatFlo, "Ankara", "Athens")
    floydWarshall(conMat, driveMatFlo, "Antalya", "Skopje")
    """
    """
    print("\nFloyd Warshall Duration (minutes)\n")

    floydWarshall(conMat, durMatFlo, "Izmir", "Sofia")
    floydWarshall(conMat, durMatFlo, "Izmir", "Athens")
    floydWarshall(conMat, durMatFlo, "Ankara", "Athens")
    floydWarshall(conMat, durMatFlo, "Antalya", "Skopje")

    print("\nDijkstra drive (km)\n")
    printDijkstra(driveMatDij,  "Izmir", "Sofia")
    printDijkstra(driveMatDij,  "Izmir", "Athens")
    printDijkstra(driveMatDij,  "Ankara", "Athens")
    printDijkstra(driveMatDij,  "Antalya", "Skopje")

    print("\nDijkstra Duration (minutes)\n")

    printDijkstra(durMatDij, "Izmir", "Sofia")
    printDijkstra(durMatDij, "Izmir", "Athens")
    printDijkstra(durMatDij, "Ankara", "Athens")
    printDijkstra(durMatDij,  "Antalya", "Skopje")

    print("\nNearestneighbour drive (km)\n")
    print(nearesneighbour(driveMat,  "Izmir", "Sofia"))
    print(nearesneighbour(driveMat,  "Izmir", "Athens"))
    print(nearesneighbour(driveMat,  "Ankara", "Athens"))
    print(nearesneighbour(driveMat,  "Antalya", "Skopje"))

    print("\nNearesneighbour Duration (minutes)\n")

    print(nearesneighbour(durMat, "Izmir", "Sofia"))
    print(nearesneighbour(durMat, "Izmir", "Athens"))
    print(nearesneighbour(durMat, "Ankara", "Athens"))
    print(nearesneighbour(durMat,  "Antalya", "Skopje"))

    print("\nhillclimb drive (km)\n")
    print(hillclimb(driveMat, airMat, "Izmir", "Sofia"))
    print(hillclimb(driveMat, airMat, "Izmir", "Athens"))
    print(hillclimb(driveMat, airMat, "Ankara", "Athens"))
    print(hillclimb(driveMat, airMat, "Antalya", "Skopje"))

    print("\nhillclimb Duration (minutes)\n")

    print(hillclimb(durMat, airMat, "Izmir", "Sofia"))
    print(hillclimb(durMat, airMat, "Izmir", "Athens"))
    print(hillclimb(durMat, airMat, "Ankara", "Athens"))
    print(hillclimb(durMat, airMat, "Antalya", "Skopje"))
    
    print("\nBest first drive (km)\n")
    
    print(printBfs(driveMatDij,"Izmir", "Sofia",airMat))
    print(printBfs(driveMatDij,"Izmir", "Athens",airMat))
    print(printBfs(driveMatDij,"Ankara", "Athens",airMat))
    print(printBfs(driveMatDij,"Antalya", "Skopje",airMat))
    
    print("\nBest first duration (minutes)\n")
    
    print(printBfs(durMatDij,"Izmir", "Sofia",airMat))
    print(printBfs(durMatDij,"Izmir", "Athens",airMat))
    print(printBfs(durMatDij,"Ankara", "Athens",airMat))
    print(printBfs(durMatDij,"Antalya", "Skopje",airMat))

    print("\nAstar drive (km)\n")

    #print(printAstar(driveMatDij,"Izmir", "Sofia",airMat))
    #print(printAstar(driveMatDij,"Izmir", "Athens",airMat))
    #print(printAstar(driveMatDij,"Ankara", "Athens",airMat))
    #print(printAstar(driveMatDij,"Antalya", "Skopje",airMat))

    print("\nAstar  duration (minutes)\n")

    #print(printAstar(durMatDij,"Izmir", "Sofia",airMat))
    #print(printAstar(durMatDij,"Izmir", "Athens",airMat))
    #print(printAstar(durMatDij,"Ankara", "Athens",airMat))
    #print(printAstar(durMatDij,"Antalya", "Skopje",airMat))

    print("\nIterative Deepening drive (km)\n")

    print(printiterdeep(driveMatDij,"Izmir", "Sofia"))
    print(printiterdeep(driveMatDij,"Izmir", "Athens"))
    print(printiterdeep(driveMatDij,"Ankara", "Athens"))
    print(printiterdeep(driveMatDij,"Antalya", "Skopje"))

    print("\nIterative Deepening duration (minutes)\n")

    print(printiterdeep(durMatDij,"Izmir", "Sofia"))
    print(printiterdeep(durMatDij,"Izmir", "Athens"))
    print(printiterdeep(durMatDij,"Ankara", "Athens"))
    print(printiterdeep(durMatDij,"Antalya", "Skopje"))

