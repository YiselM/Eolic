#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import csv
from tkinter import *
from tkinter.filedialog import askopenfilename
def readcsv(filename):
    ifile = open(filename, "r")
    reader = csv.reader(ifile, delimiter=" ")
    rownum = 0
    a = []
    x = []
    y = []
    for row in reader:
        col0 = row[0]
        hh,mm = col0.split(":")
        col0 = int(hh)*3600+int(mm)*60
        col1 = float(row[1].replace(",","."))
        col1 = (0.001723483*(col1**6)-0.04935507*(col1**5)+0.01124858*(col1**4)
                +12.34628*(col1**3)-144.3604*(col1**2)+657.3997*col1-1038.827)*(1/10)
        print(str(col0)+" "+str(col1))
        x.append(col0)
        y.append(col1)        
        rownum += 1    
    ifile.close()
    return x,y
root = Tk()
root.withdraw()
filename = askopenfilename()
root.destroy()
x,y = readcsv(filename)
print(x)
print(y)

