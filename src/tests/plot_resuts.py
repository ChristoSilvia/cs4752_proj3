#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import sys
from Tkinter import Tk
from tkFileDialog import askopenfilenames
from matplotlib.pyplot import cm 

filenames = []
if len(sys.argv[1:]) == 0:
	Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
	# filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file
	filenames = askopenfilenames()
	print filenames
	# print Tk.splitlist(filenames)
	# print(filename)
	# A = np.loadtxt(filename)

	# plt.plot(A[:,0],A[:,1],"r")
	# plt.plot(A[:,0],A[:,2],"g")
	# plt.plot(A[:,0],A[:,3],"b")
	# plt.show()
else:
	filenames = sys.argv[1:]
n = 3
color = iter(['r','g','b','y','c','k','m'])

# color=iter(cm.rainbow(np.linspace(0,1,n)))

for a_file in filenames:
	A = np.loadtxt(a_file)
	c=next(color)
	plot, = plt.plot(A[:,0],A[:,1],c=c, label=a_file)
	plt.plot(A[:,0],A[:,2],c=c)
	plt.plot(A[:,0],A[:,3],c=c)
plt.legend(loc='upper left')
plt.show()

	
