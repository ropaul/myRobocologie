
import time
from threading import Timer
import copy
import matplotlib.pyplot as plt
import numpy as np

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
   
mon_fichier1 = open("logs_400_20_900_100_5_0.2_0.0001_1_10_1_100.csv", "r")
mon_fichier2 = open("logs_400_20_900_100_5_0.2_0.0001_2_10_1_100.csv", "r")
mon_fichier3 = open("logs_400_20_900_100_5_0.2_0.0001_3_10_1_100.csv", "r")
s1 = mon_fichier1.read()
s2 = mon_fichier2.read()
s3 = mon_fichier3.read()
x = dataFromCSV(s1, 0)
print "finit x"

valy= 12

y1 = dataFromCSV(s1, valy)
y2 = dataFromCSV(s2, valy)
y3 = dataFromCSV(s3, valy)

x.pop()
y1.pop()

print len(x)
print len(y1)
print len(y2)
print len(y3)



plt.plot(x, y1, '--r',x, y2, '--b',x, y3, '--g')
plt.show()
 





      