# -*- coding: utf-8 -*-
from matplotlib.pyplot import figure, show
from scipy.stats import gaussian_kde
from numpy.random import normal
from numpy import arange
import csv 


def violin_plot2(ax,data,pos, bp=False):
    '''
    create violin plots on an axis
    '''
    dist = max(pos)-min(pos)
    w = min(0.15*max(dist,1.0),0.5)
    for d,p in zip(data,pos):
        k = gaussian_kde(d) #calculates the kernel density
        m = k.dataset.min() #lower bound of violin
        M = k.dataset.max() #upper bound of violin
        x = arange(m,M,(M-m)/100.) # support for violin
        v = k.evaluate(x) #violin profile (density curve)
        v = v/v.max()*w #scaling the violin to the available space
        ax.fill_betweenx(x,p,v+p,facecolor='#021599',alpha=0.3)
        ax.fill_betweenx(x,p,-v+p,facecolor='y',alpha=0.3)
    if bp:
        ax.boxplot(data,notch=1,positions=pos,vert=1)

# if __name__=="__main__":
#     pos = range(5)
#     data = [normal(size=100) for i in pos]
#     fig=figure()
#     ax = fig.add_subplot(111)
#     violin_plot(ax,data,pos,bp=1)
#     show()
    
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
        ax.fill_betweenx(x,p,v+p,facecolor='#990000',alpha=0.3)
        ax.fill_betweenx(x,p,-v+p,facecolor='#000099',alpha=0.3)
    ax.boxplot(data,notch=1,positions=pos,vert=1)
    show()
    return fig 
    
    
# if __name__=="__main__":
#     pos = range(5)
#     data = [normal(size=100) for i in pos]
#     print data
#     violin_plot(data,pos)
#   #  show()    
#      
# fill = csv.open("fitness_logs_1000_20_900_100_5_0.2_0.0001_1_2_1_100.csv")
# print fill
#      
     
     
     
     
     # Box plot - violin plot comparison
#
# Note that although violin plots are closely related to Tukey's (1977) box plots,
# they add useful information such as the distribution of the sample data (density trace).
#
# By default, box plots show data points outside 1.5 x the inter-quartile range as outliers
# above or below the whiskers wheras violin plots show the whole range of the data.
#
# Violin plots require matplotlib >= 1.4.

import matplotlib.pyplot as plt
import numpy as np

fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(12, 5))

# generate some random test data
all_data = [np.random.normal(0, std, 100) for std in range(6, 10)]

# plot violin plot
axes[0].violinplot(all_data,
                   showmeans=False,
                   showmedians=True)
axes[0].set_title('violin plot')

# plot box plot
axes[1].boxplot(all_data)
axes[1].set_title('box plot')

# adding horizontal grid lines
for ax in axes:
    ax.yaxis.grid(True)
    ax.set_xticks([y+1 for y in range(len(all_data))])
    ax.set_xlabel('xlabel')
    ax.set_ylabel('ylabel')

# add x-tick labels
plt.setp(axes, xticks=[y+1 for y in range(len(all_data))],
         xticklabels=['x1', 'x2', 'x3', 'x4'])
plt.show()

