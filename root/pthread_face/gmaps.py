#!/usr/bin/env python

# Gogle maps python API documentatiion
#https://github.com/googlemaps/google-maps-services-python

import urllib3.contrib.pyopenssl
import googlemaps
import requests
import pprint
#import json
import re
import simplejson as json
from gpsParser import *
import os, sys
from datetime import datetime
import time
from timeit import default_timer as timer

#import re

fpid = os.fork()
if fpid!=0:
  # Running as daemon now. PID is fpid
  sys.exit(0)
timerOn = 0   #enable for timestamps

f_ptr  = open('gmaps_log', 'w')     #redirect to prints to gmaps_log file
sys.stdout = f_ptr

invalid_escape = re.compile(r'\\[0-7]{1,6}')  # up to 6 digits for codepoints up to FFFF

def replace_with_codepoint(match):
    return unichr(int(match.group(0)[1:], 8))


def repair(brokenjson):
    return invalid_escape.sub(replace_with_codepoint, brokenjson)


urllib3.contrib.pyopenssl.inject_into_urllib3()

gmaps = googlemaps.Client(key='AIzaSyDDE19hr5RzkH7BBllLqjQMWn6C55zN4V4')

#1600 Amphitheatre Parkway, Mountain View, CA
# Geocoding an address
#geocode_result = gmaps.geocode('2995,glenwood drive apartments, 315,boulder, colorado')

#print geocode_result
# Look up an address with reverse geocoding
#reverse_geocode_result = gmaps.reverse_geocode((40.032827, -105.256697))

#print reverse_geocode_result
# Request directions via public transit
now = datetime.now()

#40.0869318, u'lng': -105.182292
currentLat = 40086932.8
currentLng = 1051825.9
# i am just concerned about this below. first arg is from address and next is to. 
directions_result = gmaps.directions("the hub apartments, boulder, colorado",
                                     "university of colorado, boulder",
                                     mode="driving",
                                     departure_time=now)
#print directions_result
# all you have to do is parse this string below. that's it. get start and end location ( address and longitide), directions( like left , right
# also take lat long info at each nodes while naviagting
#print directions_result
#print directions_result
s = str(directions_result[0]).replace("u'","\"").replace("'","\"").replace("\\xa92016","").replace("\\x","")
#s = "r'" + s + "'"
jsonGuy = json.loads(s)
print "\nStart location :" , jsonGuy['legs'][0]['start_location']
print "Start address :" , jsonGuy['legs'][0]['start_address']
print "End location :" , jsonGuy['legs'][0]['end_location']
print "End address :" , jsonGuy['legs'][0]['end_address']
print '\n'
gpsp = GpsPoller()
try:
	gpsp.start()
	f = open('gps_log', 'w')
	file_ptr = open('nav_log','w')
	id = 1
	for instruction in range(len(jsonGuy['legs'][0]['steps'])):
		file_ptr.write('Hop id '+str(id)+':')
		file_ptr.flush()
		try:
			print jsonGuy['legs'][0]['steps'][instruction]['maneuver'], " at/till " ,
		except:
			print "Get straight till :",
		print jsonGuy['legs'][0]['steps'][instruction]['end_location']
		instr = jsonGuy['legs'][0]['steps'][instruction]['end_location']
        	lat = abs(instr['lat'] * 1000000)
		lng = abs(instr['lng'] * 1000000)
		flag = True
		while True:
			#poll next hop always
			#report = gpsp.get_current_value()
                	#print report
			if timerOn == 1:
				start = timer()
			f.seek(0)   #flush everytime
                	try:
				report = gpsp.get_gps_value()
                		if report.keys()[0] == 'epx':
					if flag == True:
						print '\ncurrent lat:',report['lat']
						print 'current lon:', report['lon']
						f_ptr.flush()
					f.write( 'lat:')
                                        f.write(str(report['lat']))
                                        f.write('\nlon:')
                                        f.write(str(report['lon']))
                                        f.write('\n')
					currentLat = int(report['lat']) * 1000000
					currentLng = int(report['lon']) * 1000000
                                	time.sleep(.5)
					flag = False
                	except(AttributeError, KeyError):
                		f.write('failed\n')   #gps error
                        	time.sleep(0.5)
			f.flush()
			diffLat = abs( lat - currentLat)
			diffLng = abs( lng - currentLng)
        		#print diffLat, diffLng
			if diffLat < 300 or diffLng < 300:
				if instruction == len(jsonGuy['legs'][0]['steps']) - 1:
                        		print "Arrived at the destination"
                		else:
                        		print "Got the next hope. Routing to the next one"

				#print  "\ncurrent = [" + currentLat + ", "+currentLng+"]"
				#print  "\nnext hop was  = [" + lat + ", "+lng+"]"
				#print currentLat, currentLng, lat, lng
				file_ptr.write('reached\n')
				id += 1 
				file_ptr.flush()
				break
			if timerOn == 1:
				end = timer()
                        	print (end-start)

except:
        print "\nKilling gps_poll Thread.."
        gpsp.running = False
        gpsp.join()
        #print "\nGps device failed"
	f.close()
	file_ptr.close()
	f_ptr.close()
        sys.exit()

