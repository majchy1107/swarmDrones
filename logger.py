#!/usr/bin/python

import sys
#import math

#import tempfile
#import os

id = 1;
if __name__ == "__main__":

    infile=sys.stdin
    line=' '
    while len(line)!=0:
        line=infile.readline()
        #print line        
	    #added by Maja
	if line.startswith('LOG'):
	    #LOG id time lon lat va_lon va_lat av_norm va_course
	    line = line.replace('LOG','')
            id, time, lon, lat, va_course, dist = line.split()
	    id, time, lon, lat, va_course, dist = int(id), float(time), float(lon), float(lat), float(va_course), float(dist)
	    #print id, time, lon, lat , va_course
	    logFile_name = 'test%d.txt' % (id)
	    logFile = open(logFile_name,'a')  
	    logFile.write(line)
	    logFile.close()
	if line.startswith('LINKQ'):
	   line = line.replace('LINKQ','')
	   #neigh_id, linkq = line.split()
	   #neigh_id, linkq = int(neigh_id), float(linkq)
	   logFilelq_name = 'testlq%d.txt' % (id)
	   logFilelq = open(logFilelq_name,'a')
           logFilelq.write(line)
           logFilelq.close()
	if line.startswith('INTERACTION'): 
           line = line.replace('INTERACTION','')
           #neigh_id, linkq = line.split()
           #neigh_id, linkq = int(neigh_id), float(linkq)
           logFilelq_name = 'testint%d.txt' % (id)
           logFilelq = open(logFilelq_name,'a')
           logFilelq.write(line)
           logFilelq.close()
        if line.startswith('AUCTION'):
           line = line.replace('AUCTION','')
           logFileaut_name = 'testauction.txt'
           logFileaut = open(logFileaut_name,'a')
           logFileaut.write(line)
           logFileaut.close()
