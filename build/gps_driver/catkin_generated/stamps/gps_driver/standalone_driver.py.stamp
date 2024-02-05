
# New to python, made comments/sections on lines of code to review understanding

#=====================================================
# Step 1: Find GPGGA String 
#=====================================================

# Define the expected beginning of GPGGA string
def isGPGGAinString(inputString):
    beginning = "$GPGGA" 
    
    # Check if the GPGGA string begins with input string
    if beginning in inputString:
        # if GPGGA string found, print success message
        print('GPGGA string found! Amazing job!')
    else:
        # if GPGGA string not found, print message not found
        print ('GPGGA string not found in input string. Keep trying!')

# Example usage and sample GPGGA string received from the GNSS driver
stringReadfromPort = '$GPGGA,202530.00,5109.0262,N,11401.8407,W,5,40,0.5,1097.36,M,-17.00,M,18,TSTR*61'
isGPGGAinString(stringReadfromPort)  #Function to check if GPGGA string is present in received string.

# GPGGA string received from the GNSS driver representing geographic and positional information
gpggaRead = '$GPGGA,202530.00,5109.0262,N,11401.8407,W,5,40,0.5,1097.36,M,-17.00,M,18,TSTR*61'
gpggaSplit = gpggaRead.split(',') # Use split() without arguments to split the string based on whitespace (including commas)

#=====================================================
# Step 2: Separate fields within GPGGA String
#=====================================================

#Assign values to variables based on elements extracted from gpggaSplit list
#Ignore Index 6 (corresponds to number of satellites), Index 8 (corresponds to altitude) 

UTC = gpggaSplit[1] #Index 0 is $GPGGA, Index 1 is 202530.00 which is UTC
Latitude = gpggaSplit[2] #Index 2 is '5109.0262' the latitude
LatitudeDir = gpggaSplit[3] #Index 3 is 'N' the latitude direction 
Longitude = gpggaSplit[4] #Index 4 is '11401.8407' the longitude
LongitudeDir = gpggaSplit[5] #Index 5 is 'W' the longitude direction
HDOP = gpggaSplit[7] # Index 7 is '40' - the horizotnal dilution of precision (HDOP)

print(UTC) #printing UTC - this is a check to see if it updates effectively.

#======================================================
#Step 3: Convert Latitude & Longitude to DD.dddd format
#======================================================

def degMinstoDegDec(LatOrLong): 
    # extract the minutes and degrees from function LatOrLong
    deg = int(LatOrLong[:2]) #Retrieve the initial two characters to represent degrees. 
                             #If using DDD.mm.mm format use [:3] for three digits for degrees.
    mins = float(LatOrLong[2:]) #Extract characters starting from 3rd character to end as minutes part.

    #Express minutes as decimal degrees (representing lat and long in decimals instead of degrees, minutes/secs)
    degDec = mins/60.0

    # Sum degrees and decimal degrees
    return deg + degDec #return function used to specify value function will output when called.

#Applying latitude and longtitude valeus
LatitudeDec = degMinstoDegDec(Latitude) #Convert lat from deg & mins to decimal deg
LongitudeDec = degMinstoDegDec(Longitude)

# Sign convention for Lat and Long

def LatLongSignConvention(LatOrLong, LatOrLongDir): #Adjust sign of lat or long based on direction.
                                                    # N for pos, S for neg, E for pos, W for neg
    if LatOrLongDir == "S" or LatOrLongDir == "W":
        return -LatOrLong # South and West are negative
    else: 
        return LatOrLong # if not south and west aka north or east, print positive
    
LatitudeSigned = LatLongSignConvention(LatitudeDec, LatitudeDir) #apply sign convention based on lat- direction
LongitudeSigned = LatLongSignConvention(LongitudeDec, LongitudeDir) #apply sign convention based on long- direction

print ("Correct latitude with sign convention):",LatitudeSigned) #print correct latitude after applying sign convention
print ("Correct longitude with sign convention):", LongitudeSigned) #print correct longitude after applying sign convention

#======================================================
#Step 4: Convert Latitude & Longitude to DD.dddd format
#======================================================

import utm 

def convertToUTM(LatitudeSigned, LongitudeSigned): #Convert lat and long to UTM coords
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned) #using UTM package, does conversion

#Retrieve UTM values 
    UTMEasting = UTMVals[0] #Retreving UTM Easting value (distance eastward from UTM zone central meridian)
    UTMNorthing = UTMVals[1] #Retrieving UTM Northing value (distance northward from equator)
    UTMZone = UTMVals[2] #Retrieving UTM Zone value (UTM longitudinal zone)
    UTMLetter = UTMVals[3] #Retrieving UTM Letter value (latitude band in UTM)

    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter] #output in a vector


#Add print statements to see UTM coords
UTMcoordinates = convertToUTM(LatitudeSigned, LongitudeSigned)

print ("UTM Easting:", UTMcoordinates[0]) #should print 311355.8943456601
print ("UTM Northing:", UTMcoordinates[1]) #should print 5670013.581218471
print ("UTM Zone:", UTMcoordinates[2]) #should print 28
print ("UTM Letter:", UTMcoordinates[3]) #should print U

#======================================================
#Step 5: Convert UTC Time for ROS Applications
#======================================================

import time

def UTCtoUTCEpoch(UTC): # Retrieve current time rep in secs since the epoch ref time
    TimeSinceEpoch = int(time.mktime(time.gmtime())) #Get current time in secs since epoch in UTC using time module functions 
                                                    #based on https://docs.python.org/3/library/time.html#module-time 
    CurrentTime = TimeSinceEpoch + int(UTC) #used int instead of float since decimals would be hard to read
                                            #Convert UTC time to epoch time + add time since 1970 epoch.
    #Calculate total secs & nano-secs
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNanoSec = int((CurrentTime - CurrentTimeSec)*1e9) #Convert fractional sec to nanosec. 1 bill nanosec = 1 sec

    print("Current Time:", CurrentTime) #this prints the words current time with the current time
    return [CurrentTimeSec, CurrentTimeNanoSec]

UTC = 202530.00
CurrentTime = UTCtoUTCEpoch(UTC) #custon function to retrieve UTC time to epoch time

#======================================================
#Step 6: Serial Port -> Reading Strings
#======================================================


