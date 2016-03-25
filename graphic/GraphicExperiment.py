import matplotlib.pyplot as plt
import numpy as np


def readFile(name):
    mon_fichier1 = open(name, "r")
    s1 = mon_fichier1.read()
    y = []
    s1prime = s1.split("\n")
    s1prime.pop()
    for i in range (1,len(s1prime)):
        #print s1prime[i].split(",")
        y.append(s1prime[i].split(","))
    ynew = []
    for i in range(len(y)):
        ar = []
        for j in  range (len(y[i]) -1) :
            ar.append(float (y[i][j]) )
        ynew.append(ar)
    return ynew
    

def jumpData(data,nbJump):
    xnew = []
    ynew = []
    for i in range(len(data)):
        if (i % nbJump ==0 ):
            ar = []
            for j in  range (len(data[i]) -1) :
                ar.append(float (data[i][j]) )
            ynew.append(ar)
            xnew.append(i)
    return ynew,xnew
    


def inverser(data):
    result=[]
    temp=[]
    for i in range (len (data[0])):
        temp = []
        for j in range(len (data)):
            temp.append(data[j][i])
        result.append(temp)
    return result


def violinPloter2(data,x,title ,xlabel,ylabel ):
    plt.rc("figure",facecolor="#ffffff") # make the background white
    plt.figure() #use the fact that the background is white
    fig= plt.violinplot(ynew, showmeans=False,showmedians=True) 
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    #plt.setp(fig, xticks=[y+1 for y in range(len(ynew))], xticklabels=x)
    plt.gca().xaxis.set_ticklabels(x) #give the right number for the x label
    plt.title(title) 
    plt.autoscale(True)
    plt.show()
    
  
def violinPloter(data,x,title ,xlabel,ylabel ):  
    plt.rc("figure",facecolor="#ffffff") # make the background whit
    
    fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(12, 5)) 
    # plot violin plot
    axes.violinplot(data, showmeans=False,showmedians=True)
    axes.set_title(title)
    plt.setp(axes, xticks=[y+1 for y in range(len(data))],
             xticklabels=x)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.autoscale(True)
    plt.show()  
        



data = readFile("run80/nbgenome_logs_1000_10_900_100_5_0.2_0.0001_5_2_1_100.csv" )

data = inverser(data)

data,xnew = jumpData(data, 5)

print xnew

violinPloter(data, xnew,  " number of different genome in 80 run by generation ", "generation", "number of diferents genomes")








