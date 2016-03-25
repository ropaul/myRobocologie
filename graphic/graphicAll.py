
import time
from threading import Timer
import copy
import matplotlib.pyplot as  plt
import numpy as np
from pylab import *
from matplotlib.pyplot import figure, show
from scipy.stats import gaussian_kde
from numpy.random import normal
from numpy import arange

#give the data of the line line of a csvString which is a csv file
def dataFromCSV(csvString ,line):
    lignes = csvString.split("\n");
    result = []
    for i in range (1,len(lignes)-1):
        case = lignes[i].split(";");
        print case
        result.append(case[line])
    return result

def ploter(x , ymoy , yvar):
    taille = len(x)
    varSup = np.zeros(taille)
    varInf = np.zeros(taille)
    for i in range (taille):
        varSup[i] =ymoy[i] + yvar[i]
        varInf[i] =ymoy[i] - yvar[i]
    plt.plot(x, ymoy, 'bs', x, varSup, 'r--', x, varInf, 'r--')
    plt.show()
#    plt.plot(x, ymoy, 'bs', x, yvar, 'r--')
#    plt.show()
    plt.plot( x, yvar, 'r--')
    plt.show()     
#data is the data to show, xAxis is the x axis  and yAxis the y axis to show   
#trace violin plot
#inspired by http://pyinsci.blogspot.fr/2009/09/violin-plot-with-matplotlib.html 
def violin_plot(data, xAxis):
    fig=figure()
    ax = fig.add_subplot(111)
    '''
    create violin plots on an axis
    '''
    dist = max(xAxis)-min(xAxis)
    w = min(0.15*max(dist,1.0),0.5)
    for d,p in zip(data,xAxis):
        k = gaussian_kde(d) #calculates the kernel density
        m = k.dataset.min() #lower bound of violin
        M = k.dataset.max() #upper bound of violin
        x = arange(m,M,(M-m)/100.) # support for violin
        v = k.evaluate(x) #violin profile (density curve)
        v = v/v.max()*w #scaling the violin to the available space
        ax.fill_betweenx(x,p,v+p,facecolor='#021599',alpha=0.3)
        ax.fill_betweenx(x,p,-v+p,facecolor='y',alpha=0.3)
    #ax.boxplot(data,notch=1,positions=xAxis,vert=1)
    show()
    return fig 

   
mon_fichier1 = open("fitness_logs_1000_20_900_100_5_0.2_0.0001_1_2_1_100.csv", "r")
mon_fichier2 = open("distance_logs_1000_20_900_100_5_0.2_0.0001_1_2_1_100.csv", "r")
mon_fichier3 = open("poolSize_logs_1000_20_900_100_5_0.2_0.0001_1_2_1_100.csv", "r")
mon_fichier4 = open("genomeAge_logs_1000_20_900_100_5_0.2_0.0001_1_2_1_100.csv", "r")

s1 = mon_fichier1.read()
s2 = mon_fichier2.read()
s3 = mon_fichier3.read()
s4 = mon_fichier4.read()

#x = dataFromCSV(s1, 0)
#print "finit x"
# data = [[np.random.rand(100)] for i in range(3)]
# plt.boxplot(data)
# plt.xticks([1, 2, 3], ['mon', 'tue', 'wed'])
# plt.show()







y = []
s1prime = s1.split("\n")

s1prime.pop()
for i in range (1,len(s1prime)):
    #print s1prime[i].split(",")
    y.append(s1prime[i].split(","))


xnew = []
ynew = []

for i in range(len(y)):
    if (i % 3 ==0 ):
        ar = []
        for j in  range (len(y[i]) -1) :
        #print y[i][j]
            ar.append(float (y[i][j]) )
        ynew.append(ar)
        xnew.append(i)
        #ynew.append ([int(numeric_string) for numeric_string in y[i]])

# plt.subplot(132)
#plt.subplot(111, axisbg='#0000ab')
plt.rc("figure",facecolor="#ffffff")
#plt.figure( figsize= (10,10))
plt.figure()
fig = plt.boxplot(ynew)
plt.gca().xaxis.set_ticklabels(xnew)
#fig.patch.set_facecolor('#E0E0E0')

plt.ylim(-10, 100)
plt.xlim(0, 100)
plt.title('fitness by generations')
plt.autoscale(True)
plt.xlabel("generation ")
plt.ylabel("fitness")
plt.show()




y = []
s2prime = s2.split("\n")

s2prime.pop()
for i in range (1,len(s2prime)):
    #print s1prime[i].split(",")
    y.append(s2prime[i].split(","))


xnew = []
ynew = []

for i in range(len(y)):
    if (i % 3 ==0 ):
        ar = []
        for j in  range (len(y[i]) -1) :
        #print y[i][j]
            ar.append(float (y[i][j]) )
        ynew.append(ar)
        xnew.append(i)
        #ynew.append ([int(numeric_string) for numeric_string in y[i]])
ynew2 = []
for i in range (len(ynew)):
    ar = np.zeros(len(ynew[i]))
    for j in range (len(ynew[i])):
        ar[j]=y[i][j]
    ynew.append(ar)
print ynew2
violin_plot(ynew2,xnew)#
# plt.subplot(132)
#plt.subplot(111, axisbg='#0000ab')
plt.rc("figure",facecolor="#ffffff")
#plt.figure( figsize= (10,10))
plt.figure()

fig = plt.boxplot(ynew)
plt.gca().xaxis.set_ticklabels(xnew)
#fig.patch.set_facecolor('#E0E0E0')

plt.ylim(-10, 100)
plt.xlim(0, 100)
plt.title('distance traveled by generations')
plt.autoscale(True)
plt.xlabel("generation ")
plt.ylabel("distance traveled")
plt.show()




y = []
s3prime = s1.split("\n")

s3prime.pop()
for i in range (1,len(s3prime)):
    #print s3prime[i].split(",")
    y.append(s3prime[i].split(","))


xnew = []
ynew = []

for i in range(len(y)):
    if (i % 3 ==0 ):
        ar = []
        for j in  range (len(y[i]) -1) :
        #print y[i][j]
            ar.append(float (y[i][j]) )
        ynew.append(ar)
        xnew.append(i)
        #ynew.append ([int(numeric_string) for numeric_string in y[i]])

# plt.subplot(132)
#plt.subplot(111, axisbg='#0000ab')
plt.rc("figure",facecolor="#ffffff")
#plt.figure( figsize= (10,10))
plt.figure()
fig = plt.boxplot(ynew)
plt.gca().xaxis.set_ticklabels(xnew)
#fig.patch.set_facecolor('#E0E0E0')

plt.ylim(-10, 100)
plt.xlim(0, 100)
plt.title('number of match by generations')
plt.autoscale(True)
plt.xlabel("generation ")
plt.ylabel("number of match")
plt.show()






y = []
s4prime = s1.split("\n")

s4prime.pop()
for i in range (1,len(s4prime)):
    #print s4prime[i].split(",")
    y.append(s4prime[i].split(","))


xnew = []
ynew = []

for i in range(len(y)):
    if (i % 3 ==0 ):
        ar = []
        for j in  range (len(y[i]) -1) :
        #print y[i][j]
            ar.append(float (y[i][j]) )
        ynew.append(ar)
        xnew.append(i)
        #ynew.append ([int(numeric_string) for numeric_string in y[i]])

# plt.subplot(132)
#plt.subplot(111, axisbg='#0000ab')
plt.rc("figure",facecolor="#ffffff")
#plt.figure( figsize= (10,10))
plt.figure()
fig = plt.boxplot(ynew)
plt.gca().xaxis.set_ticklabels(xnew)
#fig.patch.set_facecolor('#E0E0E0')

plt.ylim(-10, 100)
plt.xlim(0, 100)
plt.title('time of generaction of genomes by generations')
plt.autoscale(True)
plt.xlabel("generation ")
plt.ylabel("time of generaction of genomes ")
plt.show()

