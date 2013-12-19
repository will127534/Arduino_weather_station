#!/usr/bin/python
import subprocess
import re
import sys
import time
import datetime
import gspread
import serial
import logging
import os
import csv
import datetime
import math
from time import gmtime, strftime
import eeml.datastream
from eeml.datastream import CosmError
global t
#----------------------------------google Doc setting
email =''
pas = ''
spread = ''
try:
    gc = gspread.login(email, pas)
except:
    print "Unable to log in.  Check your email address/password"
    
try:
    worksheet = gc.open(spread).sheet1
except:
    print ("Unable to open the spreadsheet.  Check your filename: %s" & spread)
    spread = input('>>Enter new spreadsheet name ')
    
 
#---------------------------------Cosm setting
API_KEY = ''
FEED = 
API_URL = '/v2/feeds/{feednum}.xml' .format(feednum = FEED)
pac = eeml.datastream.Cosm(API_URL, API_KEY)

t = time.time()
#---------------------------------CSV log setting
logpath = os.getcwd() + "/sensordata.csv"
logging.basicConfig(format='%(asctime)s,%(message)s',datefmt='%m/%d/%Y %I:%M:%S', filename = logpath)
log = logging.getLogger('error')

global humidity         # Using 
global temp
global dew 
global BMPpressure
global batteryvoltage
global Batteryleftmah


global dtemp1            # Not using now 
global dtemp2
global batterytemp
global light
global plant
global BMPtemp
global error
global BH1750
global Batterycurrent
global spendtime

humidity =0
temp=0
dew =0
BMPpressure=0
batteryvoltage=0
Batteryleftmah=0
dtemp1=0
dtemp2=0
batterytemp=0
light=0
plant=0
BMPtemp=0
error=0
BH1750=0
Batterycurrent=0
spendtime=0

#------------------Open serial port 
error = 1
while error == 1:
  try:
    s=serial.Serial("/dev/ttyAMA0",9600,timeout=None,writeTimeout=2)
    print('Serial started,start listening')
    error = 0    
  except Exception:
    print "SerialException"

    
    error = 1
    time.sleep(1)

  
flag = s.isOpen()

#-----------------Commuication statistics 
global Networkerror 
global networkerror 
global Dataerror 
global dataerror 
global totaldata 
Networkerror = 0
networkerror = 0
Dataerror = 0
dataerror = 0
totaldata = 0


def dewPoint(A,B):               # Dew point output A(Tempeture C) B(Humidity)
  RATIO = 373.15 / (273.15 + A)
  RHS = -7.90298 * (RATIO - 1)
  RHS += 5.02808 * math.log(RATIO,10)
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) 
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) 
  RHS += math.log(1013.246,10)
  VP = pow(10, RHS - 3) * B;
  T = math.log(VP/0.61078);   
  return (241.88 * T) / (17.558 - T)

def update():                    # Cosm PUT
        global Networkerror 
        try:
            pac.put()
            print ('Cosm Update Complete')
        except:
            print('Network error')
            Networkerror = Networkerror + 1
            update()

def csvwrite():                 
    global humidity         # Using 
    global temp
    global dew 
    global BMPpressure
    global batteryvoltage
    global Batteryleftmah

    try:
        log.error("%s,%s,%s,%s,%s" % (batteryvoltage ,temp,humidity ,Batteryleftmah, BMPpressure))
        print('Finish writing csv file')
    except:
        print("Error writing csv file")
        csvwrite()    # Retry
        return
def gdocwrite():                  
    global humidity         # Using 
    global temp
    global dew 
    global BMPpressure
    global batteryvoltage
    global Batteryleftmah
    global t

    try:
        values = [t,batteryvoltage ,temp ,humidity ,dew , BMPpressure]
        worksheet.append_row(values)
        print "Google Docs updated "
    except:
        print "Unable to append data.  Check your connection?"
        gdocwrite()

def read():                
        global Networkerror 
        global networkerror 
        global Dataerror 
        global dataerror 
        global totaldata
        global humidity
        global temp
        global dew
        global BMPpressure
        global batteryvoltage
        global Batteryleftmah
        global t

        data = ''                      # Clear data buffer  
        data = s.read(1)               # This will block until one more char or timeout
        
        time.sleep(0.1)                # Wait for all data
        data += s.read(s.inWaiting())  # Reading all data in buffer 
                             
        totaldata = totaldata +1       
        
        data = data.split(',')         # Input data is CSV type
        print(data)
        try:
            Batteryleftmah=  data[4]
            Batteryleftmah = Batteryleftmah.rstrip('\n').rstrip('\r')
            humidity = data[0]
            temp =  data[1]
            BMPpressure =  data[2]
            batteryvoltage=  data[3]
                                 
            t = time.strftime(" %H:%M:%S ", gmtime())
            print ('----------------%s-----------------' % t)
            dew = dewPoint(float(temp),float(humidity))
            print('Data Get: Temp:%s Humidity:%s%% Dew point:%0.1f Pressure:%shPa Voltage:%sV Battery left:%smah' % (temp,humidity,dew,BMPpressure,batteryvoltage,Batteryleftmah))
            
        except:
            print('split/rstrip error')
            dataerror = dataerror + 1
            read()                           
            return
        #---------------Calculate commuication statistics 
        Dataerror = float(dataerror)/float(totaldata)*100.
        networkerror = float(Networkerror)/float(totaldata)*100.
        print('Count:%s Data error count:%s  %.2f%%  Network error count:%s %.2f%%' % (totaldata,dataerror,Dataerror,Networkerror,networkerror) )
        

        pac.update([eeml.Data(1, temp)])
        pac.update([eeml.Data(4, humidity)])
        pac.update([eeml.Data(10,BMPpressure)])
        pac.update([eeml.Data(6, Batteryleftmah)])
        pac.update([eeml.Data(0, batteryvoltage)])
        pac.update([eeml.Data(5, dew)])
      
        update()     # Cosm
        csvwrite()   # Csv log
        gdocwrite()  # Google Docs  




while 1:
    s.flush()
    read()
       
    
    

  


