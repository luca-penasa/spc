#!/usr/bin/python
import sys
import numpy as np

import re
datefield = re.compile("\[.*.\]")
stringfield = re.compile("^[a-zA-Z]")
floatfield = re.compile(".*[0-9]*\.?[0-9].*")

n_rows = 4

filename = sys.argv[1]

print("found file " + filename)

thisfile = open(filename, 'r')

matrixes = []
values = []
for line in thisfile:    
    if line == '\n':
        continue
        
    if datefield.match(line):
        #in this case we clean the line
        line = datefield.split(line)[1]        
        print "matched:" + line 
        
    if stringfield.match(line):
        print "discarded line:" + line
        continue
        
        
        
    #now split the string and get the numbers
    line= line.replace('\n', '') #remove endline
    
    qua = line.split()
    if len(qua) < n_rows:
        qua = line.split(' ') #try awhite space

    for val in qua:
        values.append(float(val))
        
    if len(values) == n_rows * n_rows:
        mat = np.matrix(values).reshape((n_rows, n_rows))
        values = []
        matrixes.append(mat)

for m in matrixes:
    print ("MAT:")
    print m
    print "\n"

T = np.eye(n_rows)  * 10
T[3,3] = 1.0           
for m in matrixes:
    T = m * T 

### scaling
#scalemat = np.eye(n_rows) * 10

#scalemat[3, 3] = 1.0
#T = scalemat * T

#T = np.eye(n_rows) * 10 * T

print "final T matrix:"
print T
print ("\n")
for row in np.array(T):
    print("{}\t{}\t{}\t{}".format(row[0], row[1], row[2], row[3]))

                
