#! /usr/bin/python
 
from gps import *
import time
import os
    
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE) 
   
try:
    while True:
         
        report = gpsd.next() #
        if report['class'] == 'SKY':
            os.system('clear')
            print ' Satellites (total of', len(gpsd.satellites) , ' in view)'
            for i in gpsd.satellites:
                print 't', i
 
         
            print '\n\n'
            print 'PRN = PRN ID of the satellite. 1-63 are GNSS satellites, 64-96 are GLONASS satellites, 100-164 are SBAS satellites'
            print 'E = Elevation in degrees'
            print 'As = Azimuth, degrees from true north'
            print 'ss = Signal stength in dB'
            print 'used = Used in current solution?'
 
        time.sleep(1) 
 
 
except (KeyboardInterrupt, SystemExit): #when you press ctrl+c
    print "Done.\nExiting."

