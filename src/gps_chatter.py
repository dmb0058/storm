#! /usr/bin/python
 
from gps import *
import time
import os
    
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE) 
   
try:
    while True:
         
        report = gpsd.next() #
	print report
        time.sleep(1) 
 
 
except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "Done.\nExiting."

