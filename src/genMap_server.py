#!/usr/bin/python
# Python program to get a google map image of specified location using Google Static Maps API 
  
# importing required modules 
import requests 
import rospy
from storm.srv import *


def generate_map(req):
	api_key = "AIzaSyADmZA4pWWXUzKfmwWwLrsPlrzV7XtrAJo" 
	url = "https://maps.googleapis.com/maps/api/staticmap?"
 	 
	# zoom defines the zoom level of the map 
	zoom = 20
 	 
	# maptype is roadmap, satellite, terrain or hybrid
	maptype = "hybrid"

	# get method of requests module return response object 

	# center defines the center of the map, equidistant from all edges of the map.  
	center = str(req.latitude) + "," + str(req.longitude)
	r = requests.get(url + "center=" + center + "&maptype=" + maptype + "&zoom=" + str(zoom) + "&size=640x640&key=" + api_key + "&sensor=false") 
  
	# wb mode is stand for write binary mode 
	f = open('/var/www/html/map.png', 'wb') 
  
	# r.content gives content, 
	# in this case gives image 
	f.write(r.content) 
  
	# close mthod of file object 
	# save and close the file 
	f.close()
	return genMapResponse(1)


# advertise the service - name, type, callback function

rospy.init_node('genMap_server')
service = rospy.Service('genMap', genMap, generate_map)

rospy.spin()

