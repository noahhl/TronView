#!/usr/bin/env python

# Serial input source
# Garmin G3X
# 01/30/2019 Brian Chesteen, credit to Christopher Jones for developing template for input modules.
# 06/18/2023 C.Jones fix for live serial data input.

from ._input import Input
from lib import hud_utils
from lib import hud_text
from lib import geomag
from . import _utils
import serial
import struct
import math, sys
import time


class serial_g3x(Input):
    def __init__(self):
        self.name = "g3x"
        self.version = 1.1
        self.inputtype = "serial"

        # Setup moving averages to smooth a bit
        self.readings = []
        self.max_samples = 10
        self.vsi_max_samples = 50
        self.vsi_readings = []
        self.pitch_max_samples = 10
        self.pitch_readings = []
        self.readings1 = []
        self.max_samples1 = 20
        self.EOL = 10

    def initInput(self,num, aircraft):
        aircraft.setDataMeasurementFormat(1)
        Input.initInput(self,num, aircraft)  # call parent init Input.
        if(aircraft.inputs[self.inputNum].PlayFile!=None):
            # play a log file?
            if aircraft.inputs[self.inputNum].PlayFile==True:
                defaultTo = "garmin_g3x_data1.txt"
                aircraft.inputs[self.inputNum].PlayFile = hud_utils.readConfig(self.name, "playback_file", defaultTo)
            self.ser,self.input_logFileName = Input.openLogFile(self,aircraft.inputs[self.inputNum].PlayFile,"r")
            self.isPlaybackMode = True
        else:
            self.efis_data_format = hud_utils.readConfig("DataInput", "format", "none")
            self.efis_data_port = hud_utils.readConfig(
                "DataInput", "port", "/dev/ttyS0"
            )
            self.efis_data_baudrate = hud_utils.readConfigInt(
                "DataInput", "baudrate", 115200
            )
            # open serial connection.
            self.ser = serial.Serial(
                port=self.efis_data_port,
                baudrate=self.efis_data_baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1,
            )

        # check for system platform??
        #if sys.platform.startswith('win'):
        #    self.EOL = 10
        #else:
        #    self.EOL = 13
        self.EOL = 13

    # close this input source
    def closeInput(self, aircraft):
        if self.isPlaybackMode:
            self.ser.close()
        else:
            self.ser.close()

    #############################################
    ## Function: readMessage
    def readMessage(self, aircraft):
        def mean(nums):
            return float(sum(nums)) / max(len(nums), 1)

        if aircraft.errorFoundNeedToExit:
            return aircraft
        try:
            x = 0
            while x != 61:  # 61(=) is start of garmin g3x sentence.
                t = self.ser.read(1)
                if len(t) != 0:
                    x = ord(t)
                    if x == 64:  # 64(@) is start of garmin g3x GPS sentence.
                        msg = self.ser.read(56)
                        if(isinstance(msg,str)): msg = msg.encode() # if read from file then convert to bytes
                        aircraft.msg_last = msg
                        if len(msg) == 56:
                            UTCYear, UTCMonth, UTCDay, UTCHour, UTCMin, UTCSec, LatHemi, LatDeg, LatMin, LonHemi, LonDeg, LonMin, PosStat, HPE, GPSAlt, EWVelDir, EWVelmag, NSVelDir, NSVelmag, VVelDir, VVelmag, CRLF = struct.unpack(
                                "2s2s2s2s2s2sc2s5sc3s5sc3s6sc4sc4sc4s2s", msg
                            )
                            aircraft.msg_count += 1
                            aircraft.sys_time_string = "%02d:%02d:%02d"%(int(UTCHour),int(UTCMin),int(UTCSec))
                            self.time_stamp_string = aircraft.sys_time_string
                            self.time_stamp_min = int(UTCMin)
                            self.time_stamp_sec = int(UTCSec)
                            aircraft.gps.LatHemi = LatHemi.decode()# North or South
                            aircraft.gps.LatDeg = int(LatDeg)
                            aircraft.gps.LatMin = int(LatMin) * 0.001  # x.xxx
                            aircraft.gps.LonHemi = LonHemi.decode()# East or West
                            aircraft.gps.LonDeg = int(LonDeg)
                            aircraft.gps.LonMin = int(LonMin) * 0.001  # x.xxx
                            aircraft.gps.GPSAlt = int(GPSAlt) * 3.28084
                            aircraft.gps.EWVelDir = EWVelDir.decode() # E or W
                            aircraft.gps.EWVelmag = int(EWVelmag) * 0.1
                            aircraft.gps.NSVelDir = NSVelDir.decode() # N or S
                            aircraft.gps.NSVelmag = int(NSVelmag) * 0.1
                            aircraft.gps.VVelDir = VVelDir.decode()  # U or D
                            aircraft.gps.VVelmag = int(VVelmag) * 0.01 * (-1 if aircraft.gps.VVelDir == 'D' else 1 )
                            aircraft.mag_decl = _utils.geomag(
                                aircraft.gps.LatHemi,
                                aircraft.gps.LatDeg,
                                aircraft.gps.LatMin,
                                aircraft.gps.LonHemi,
                                aircraft.gps.LonDeg,
                                aircraft.gps.LonMin,
                            )
                            aircraft.gndtrack = _utils.gndtrack(
                                aircraft.gps.EWVelDir, aircraft.gps.EWVelmag, aircraft.gps.NSVelDir, aircraft.gps.NSVelmag
                            )
                            #aircraft.gndangle = math.degrees(math.atan2(aircraft.gps.VVelmag * 2.23694, aircraft.gndspeed))
                            aircraft.gndangle = math.degrees(math.atan2(aircraft.vsi * 0.0113636 , aircraft.gndspeed))
                            aircraft.gps.Source = "G3X"
                            aircraft.wind_speed, aircraft.wind_dir, aircraft.norm_wind_dir = _utils.windSpdDir(
                                aircraft.tas * 0.8689758, # back to knots.
                                aircraft.gndspeed * 0.8689758, # convert back to knots
                                aircraft.gndtrack,
                                aircraft.mag_head,
                                aircraft.mag_decl,
                            )
                            if self.output_logFile != None:
                                Input.addToLog(self,self.output_logFile,bytes([64]))
                                Input.addToLog(self,self.output_logFile,msg)

                            aircraft.gps.msg_count += 1

                else:
                    if (self.isPlaybackMode ):  # if no bytes read and in playback mode.  then reset the file pointer to the start of the file.
                        self.ser.seek(0)
                    return aircraft

            id = self.ser.read(1)
            if id == b'C' or id =='C':
                SentID = 0
            else:
                SentID = int(id)
            if SentID == 1:  # atittude/air data message
                msg = self.ser.read(57)
                aircraft.msg_last = msg
                if len(msg) == 57:
                    if(isinstance(msg,str)): msg = msg.encode() # if read from file then convert to bytes
                    SentVer, UTCHour, UTCMin, UTCSec, UTCSecFrac, Pitch, Roll, Heading, Airspeed, PressAlt, RateofTurn, LatAcc, VertAcc, AOA, VertSpeed, OAT, AltSet, Checksum, CRLF = struct.unpack(
                        "c2s2s2s2s4s5s3s4s6s4s3s3s2s4s3s3s2s2s", msg
                    )
                    if int(SentVer) == 1 :
                        aircraft.sys_time_string = "%02d:%02d:%02d"%(int(UTCHour),int(UTCMin),int(UTCSec))
                        aircraft.roll = int(Roll) * 0.1 if _utils.is_number(Roll)  else 0
                        if _utils.is_number(Pitch):
                            self.pitch_readings.append(int(Pitch) * 0.1) 
                            aircraft.pitch = mean(self.pitch_readings)
                        if len(self.pitch_readings) == self.pitch_max_samples:
                            self.pitch_readings.pop(0)
                        aircraft.ias = int(Airspeed) * 0.1  *  1.15078 if Airspeed != b'____' else 0
                        aircraft.PALT = int(PressAlt) if _utils.is_number(PressAlt)  else 0
                        aircraft.oat = (int(OAT) * 1.8) + 32 if _utils.is_number(OAT)  else 0 # c to f
                        if _utils.is_number(AOA) == True:
                            aircraft.aoa = int(AOA)
                            self.readings1.append(aircraft.aoa)
                            aircraft.aoa = mean(
                                self.readings1
                            )  # Moving average to smooth a bit
                        else:
                            aircraft.aoa = 0
                        if len(self.readings1) == self.max_samples1:
                            self.readings1.pop(0)
                        aircraft.mag_head = int(Heading) if _utils.is_number(Heading)  else 0 
                        aircraft.baro = (int(AltSet) + 2750.0) / 100.0 if _utils.is_number(AltSet) else 0 
                        aircraft.baro_diff = aircraft.baro - 29.9213
                        aircraft.alt = int(
                            aircraft.PALT  + (aircraft.baro_diff / 0.00108)
                        )  # 0.00108 of inches of mercury change per foot.
                        aircraft.BALT = aircraft.alt
                        if _utils.is_number(VertSpeed):
                            self.vsi_readings.append(int(VertSpeed) )
                            aircraft.vsi = round(mean(self.vsi_readings ), 0)  * 10
                        if len(self.vsi_readings) == self.vsi_max_samples:
                            self.vsi_readings.pop(0)
                        aircraft.turn_rate = int(RateofTurn) * 0.1 if _utils.is_number(RateofTurn) else 0
                        aircraft.vert_G = int(VertAcc) * 0.1 if _utils.is_number(VertAcc) else 0
                        aircraft.slip_skid = int(LatAcc) * 0.01 if _utils.is_number(LatAcc) else 0
                        self.readings.append(aircraft.slip_skid)
                        aircraft.slip_skid = mean(
                            self.readings
                        )  # Moving average to smooth a bit
                        if len(self.readings) == self.max_samples:
                            self.readings.pop(0)
                        aircraft.msg_count += 1
                        if (self.isPlaybackMode):  # if playback mode then add a delay.  Else reading a file is way to fast.
                            time.sleep(0.01)

                        if self.output_logFile != None:
                            #Input.addToLog(self,self.output_logFile,bytes([61,ord(SentID)]))
                            Input.addToLog(self,self.output_logFile,msg)


                    else:
                        aircraft.msg_bad += 1

                else:
                    aircraft.msg_bad += 1

            elif SentID == 2:
                msg = self.ser.read(40)
                if(isinstance(msg,str)): msg = msg.encode() # if read from file then convert to bytes
                aircraft.msg_last = msg
                if len(msg) == 40:
                    SentVer, UTCHour, UTCMin, UTCSec, UTCSecFrac, TAS, DensityAlt, HeadingBug, AltitudeBug, AirspeedBug, VerticalSpeedBug,Checksum, CRLF = struct.unpack(
                            "c2s2s2s2s4s6s3s6s4s4s2s2s", msg)
                    if int(SentVer) == 1:
                        aircraft.sys_time_string = "%02d:%02d:%02d"%(int(UTCHour),int(UTCMin),int(UTCSec))
                        aircraft.DA = int(DensityAlt) if _utils.is_number(DensityAlt) else 0 
                        aircraft.tas = int(TAS) * 0.115078 if _utils.is_number(TAS) else 0
                        aircraft.nav.HeadBug = int(HeadingBug) if _utils.is_number(HeadingBug) else 0
                        aircraft.nav.AltBug = int(AltitudeBug) if _utils.is_number(AltitudeBug) else 0
                    else:
                        aircraft.msg_bad += 1
            elif SentID == 7:  # GPS AGL data message
                msg = self.ser.read(20)
                if(isinstance(msg,str)): msg = msg.encode() # if read from file then convert to bytes
                aircraft.msg_last = msg
                if len(msg) == 20 :
                    msg = (msg[:20]) if len(msg) > 20 else msg
                    SentVer, UTCHour, UTCMin, UTCSec, UTCSecFrac, HeightAGL, Groundspeed, Checksum, CRLF = struct.unpack(
                        "c2s2s2s2s3s4s2s2s", msg
                    )
                    if int(SentVer) == 1 :
                        aircraft.sys_time_string = "%02d:%02d:%02d"%(int(UTCHour),int(UTCMin),int(UTCSec))
                        aircraft.agl = int(HeightAGL) * 100 if _utils.is_number(HeightAGL) else 0
                        aircraft.gndspeed = int(Groundspeed) * 0.115078 if _utils.is_number(Groundspeed) else 0  #in mph
                        aircraft.msg_count += 1
                        if self.output_logFile != None:
                            #Input.addToLog(self,self.output_logFile,bytes([61,ord(SentID)]))
                            Input.addToLog(self,self.output_logFile,msg)

                    else:
                        aircraft.msg_bad += 1
                        aircraft.debug1 = "bad GPS AGL data - unkown ver"

                else:
                    aircraft.msg_bad += 1
                    aircraft.debug1 = "bad GPS AGL data - wrong length"

            elif SentID == 3: #Engine data message
                msg = self.ser.read(219)
                if(isinstance(msg,str)): msg = msg.encode()
                aircraft.engine.msg_last = msg
                if len(msg) == 219:
                    SentVer, UTCHour, UTCMin, UTCSec, UTCSecFrac,OilP, OilT, RPM, Unused,MP,FF,Unused2,FP,FQ1,FQ2,FRem,V1, V2, A1, HobbsT, TachT, CHT6, EGT6, CHT5, EGT5, CHT4, EGT4, CHT3, EGT3, CHT2,EGT2,CHT1, EGT1, TIT1, TIT2, ElevTrim, ElevUnits, FlapsP, FlapsUnits, CarbT, CarbUnits, CoolantP, CooalntUnits, CoolantT, CoolantTUnits, A2, AmpsUnits, AileronTrim, AileronUnits, RudderTrim, RuddderUnits, FQ3, FQ3Units, FQ4, FQ4Units, Unused3, Discrete1, Disrete2, Discrete3, Discrete4, Unused4, Checksum, CRLF = struct.unpack(
                            "c2s2s2s2s3s4s4s4s3s3s3s3s3s3s3s3s3s4s5s5s4s4s4s4s4s4s4s4s4s4s4s4s4s4s5s1s5s1s5s1s5s1s5s1s5s1s5s1s5s1s5s1s5s1s18s1s1s1s1s12s2s2s", msg
                            )
                    aircraft.sys_time_string = "%02d:%02d:%02d"%(int(UTCHour),int(UTCMin),int(UTCSec))
                    aircraft.engine.RPM = int(RPM) if _utils.is_number(RPM) else 0 
                    aircraft.engine.NumberOfCylinders = 4
                    aircraft.engine.OilPress = int(OilP) if _utils.is_number(OilP) else 0
                    aircraft.engine.FuelPress = int(FP) if _utils.is_number(FP) else 0
                    aircraft.engine.CoolantTemp = 32 + int(CoolantT) * 9/50.0 if _utils.is_number(CoolantT) else 0
                    aircraft.engine.OilTemp = 32+ int(OilT) * 9/5.0 if _utils.is_number(OilT) else 0
                    aircraft.engine.FuelFlow = int(FF) / 10.0 if _utils.is_number(FF) else 0
                    aircraft.engine.ManPress = int(MP) /10.0 if _utils.is_number(MP) else 0

                    for i,x in enumerate([EGT1,EGT2,EGT3,EGT4,EGT5,EGT6]):
                        aircraft.engine.EGT[i] = 32+ int(x) * 9/5.0 if _utils.is_number(x) else 0
                    for i,x in enumerate([CHT1,CHT2,CHT3,CHT4,CHT5,CHT6]):
                        aircraft.engine.CHT[i] = 32 + int(x) * 1.8 if _utils.is_number(x) else 0

                    aircraft.fuel.FuelLevels[0] = int(FQ1) / 10.0 if _utils.is_number(FQ1) else 0 
                    aircraft.fuel.FuelLevels[1] = int(FQ2) / 10.0 if _utils.is_number(FQ2) else 0 
                    aircraft.fuel.FuelLevels[2] = int(FQ3) / 10.0 if _utils.is_number(FQ3) else 0 
                    aircraft.fuel.FuelLevels[3] = int(FQ4) / 10.0 if _utils.is_number(FQ4) else 0 
                else:
                    aircraft.engine.msg_bad += 1
                    aircraft.engine.debug1 = "bad engine data - wrong length"

            else:
                aircraft.debug2 = SentID
                aircraft.msg_unknown += 1  # else unknown message.
                if self.isPlaybackMode:
                    time.sleep(0.01)
                else:
                    self.ser.flushInput()  # flush the serial after every message else we see delays
                return aircraft

        except serial.serialutil.SerialException as e:
            print(e)
            print("G3X serial exception")
            aircraft.errorFoundNeedToExit = True

        return aircraft




# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python
