'''
    #TODO:
    Copyright (C) 2013  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Laboratory, Boston University

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

from __future__ import with_statement # for Python 2.5

import time
from threading import RLock, Thread

import numpy as np

from comm import Protocol, UDPIPInterface, DataID, SysConf


class Platform(Thread):
    '''
    classdocs
    '''


    def __init__(self, hostname, portno):
        '''
        Constructor
        '''
        Thread.__init__(self)
        
        # connect to hostname and portno
        self.interface = UDPIPInterface(hostname, portno)
        self.interface.connect()
        
        self.protocol = Protocol()
        
        # Lock for platform state
        self._lock = RLock()
        self.running = True
        
        # platform state
        self.packageNumber = 0xFF
        self.us = [0] * self.protocol.NUS
        self.ir = 0
        self.humanSensor = [0] * 4 # TODO: replace with named constant
        self.tilt = [0, 0] # tilting on X, Y axis
        self.temp = [0, 0, 0] # temperature sensor, overhead AD1, AD2
        self.irCode = [0, 0, 0, 0] # 4 byte of the received ir code
        self.customAD = [0] * self.protocol.NCUSTOMCH
        self.voltage = [0, 0] # reference and potentiometer supply voltage
        self.potentiometer = [0] * self.protocol.NMOTORS
        self.motorCurrent = [0] * self.protocol.NMOTORS
        self.encoderPosition = [0] * 2 # TODO: replace with named constant
        self.encoderSpeed = [0] * 2 # TODO: replace with named constant
        self.encoderDirection = [0] * 2 # TODO: replace with named constant
        self.customAD = [0] * self.protocol.NCUSTOMCH
        self.gpio = 0
        
        self.battery = [0, 0, 0] # TODO: figure out if this is the same as
        # customAD[0:3]
    
    def __del__(self):
        '''
        Destructor
        '''
        self.interface.disconnect()

#-------------------------------------------------------------------------------
#TODO: 99/111 DrRobot legacy API
# 4 Deprecated, 1 Future, 5 Undocumented

    #---------------------------------- 12/16
    def SystemMotorSensorRequest(self, PackageNumber):
        with self._lock:
            self.packageNumber = PackageNumber
            self.protocol.requestSensorData(DataID.motorSgn, PackageNumber)
            self.interface.sendCommand(self.protocol.createRequest())
        
    
    def SystemStandardSensorRequest(self, PackageNumber):
        with self._lock:
            self.packageNumber = PackageNumber
            self.protocol.requestSensorData(DataID.sensor, PackageNumber)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def SystemCustomSensorRequest(self, PackageNumber):
        with self._lock:
            self.packageNumber = PackageNumber
            self.protocol.requestSensorData(DataID.ADC, PackageNumber)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def SystemAllSensorRequest(self, PackageNumber):
        with self._lock:
            self.packageNumber = PackageNumber
            self.protocol.requestSensorData(DataID.allSensor, PackageNumber)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def EnableMotorSensorSending(self):
        self.SystemMotorSensorRequest(self.packageNumber)
    
    def EnableStandardSensorSending(self):
        self.SystemStandardSensorRequest(self.packageNumber)
    
    def EnableCustomSensorSending(self):
        self.SystemCustomSensorRequest(self.packageNumber)
    
    def EnableAllSensorSending(self):
        self.SystemAllSensorRequest(self.packageNumber)
 
    def DisableMotorSensorSending(self):
        self.SystemMotorSensorRequest(0)
    
    def DisableStandardSensorSending(self):
        self.SystemStandardSensorRequest(0)
    
    def DisableCustomSensorSending(self):
        self.SystemCustomSensorRequest(0)
    
    def DisableAllSensorSending(self):
        self.SystemAllSensorRequest(0)
 
    def SetSysMotorSensorPeriod(self, PeriodTime):
        raise NotImplementedError # TODO:
    
    def SetSysStandardSensorPeriod(self, PeriodTime):
        raise NotImplementedError # TODO:
    
    def SetSysCustomSensorPeriod(self, PeriodTime):
        raise NotImplementedError # TODO:
    
    def SetSysAllSensorPeriod(self, PeriodTime):
        raise NotImplementedError # TODO:

    #---------------------------------- 8/8
    
    def GetSensorSonar1(self):
        return self.GetSensorSonar(0)
    
    def GetSensorSonar2(self):
        return self.GetSensorSonar(1)
    
    def GetSensorSonar3(self):
        return self.GetSensorSonar(2)
    
    def GetSensorSonar4(self):
        return self.GetSensorSonar(3)
    
    def GetSensorSonar5(self):
        return self.GetSensorSonar(4)
    
    def GetSensorSonar6(self):
        return self.GetSensorSonar(5)
    
    def GetSensorSonar(self, channel):
        with self._lock:
            return self.us[channel]
    
    def GetSensorIRRange(self):
        with self._lock:
            return self.ir
    
    #---------------------------------- 4/4
    
    def GetSensorHumanAlarm1(self):
        with self._lock:
            return self.humanSensor[0]
    
    def GetSensorHumanAlarm2(self):
        with self._lock:
            return self.humanSensor[1]
    
    def GetSensorHumanMotion1(self):
        with self._lock:
            return self.humanSensor[2]
    
    def GetSensorHumanMotion2(self):
        with self._lock:
            return self.humanSensor[3]
    
    #---------------------------------- 2/2
    
    def GetSensorTiltingX(self):
        with self._lock:
            return self.tilt[0]
    
    def GetSensorTiltingY(self):
        with self._lock:
            return self.tilt[1]
    
    #---------------------------------- 3/3
    
    def GetSensorOverheatAD1(self):
        with self._lock:
            return self.temp[1]
    
    def GetSensorOverheatAD2(self):
        with self._lock:
            return self.temp[2]
    
    def GetSensorTemperature(self):
        with self._lock:
            return self.temp[0]
    
    #---------------------------------- 4/5
    
    def GetSensorIRCode1(self):
        with self._lock:
            return self.irCode[0]
    
    def GetSensorIRCode2(self):
        with self._lock:
            return self.irCode[1]
    
    def GetSensorIRCode3(self):
        with self._lock:
            return self.irCode[2]
    
    def GetSensorIRCode4(self):
        with self._lock:
            return self.irCode[3]
    
    def SetInfraredControlOutput(self, LowWord, HighWord):
        raise NotImplementedError # TODO:
    
    #---------------------------------- 5/5
    
    def GetSensorBatteryAD1(self):
        return self.GetCustomAD1()
    
    def GetSensorBatteryAD2(self):
        return self.GetCustomAD2()
    
    def GetSensorBatteryAD3(self):
        return self.GetCustomAD3()
    
    def GetSensorRefVoltage(self):
        with self._lock:
            return self.voltage[0]
    
    def GetSensorPotVoltage(self):
        with self._lock:
            return self.voltage[1]
    
    #---------------------------------- 7/7
    
    def GetSensorPot1(self):
        return self.GetSensorPot(0)
    
    def GetSensorPot2(self):
        return self.GetSensorPot(1)
    
    def GetSensorPot3(self):
        return self.GetSensorPot(2)
    
    def GetSensorPot4(self):
        return self.GetSensorPot(3)
    
    def GetSensorPot5(self):
        return self.GetSensorPot(4)
    
    def GetSensorPot6(self):
        return self.GetSensorPot(5)
    
    def GetSensorPot(self, channel):
        with self._lock:
            return self.potetiometer[channel]
    
    #---------------------------------- 7/7
    
    def GetMotorCurrent1(self):
        return self.GetMotorCurrent(0)
    
    def GetMotorCurrent2(self):
        return self.GetMotorCurrent(1)
    
    def GetMotorCurrent3(self):
        return self.GetMotorCurrent(2)
    
    def GetMotorCurrent4(self):
        return self.GetMotorCurrent(3)
    
    def GetMotorCurrent5(self):
        return self.GetMotorCurrent(4)
    
    def GetMotorCurrent6(self):
        return self.GetMotorCurrent(4)
    
    def GetMotorCurrent(self, channel):
        with self._lock:
            return self.motorCurrent[channel]
    
    #---------------------------------- 6/6
    
    def GetEncoderDir1(self):
        with self._lock:
            return self.encoderDirection[0]
    
    def GetEncoderDir2(self):
        with self._lock:
            return self.encoderDirection[1]
    
    def GetEncoderPulse1(self):
        with self._lock:
            return self.encoderPosition[0]
    
    def GetEncoderPulse2(self):
        with self._lock:
            return self.encoderPosition[1]
    
    def GetEncoderSpeed1(self):
        with self._lock:
            return self.encoderSpeed[0]
    
    def GetEncoderSpeed2(self):
        with self._lock:
            return self.encoderSpeed[1]
    
    #---------------------------------- 11/11
    
    def GetCustomAD1(self):
        return self.GetCustomAD(0)
    
    def GetCustomAD2(self):
        return self.GetCustomAD(1)
    
    def GetCustomAD3(self):
        return self.GetCustomAD(2)
    
    def GetCustomAD4(self):
        return self.GetCustomAD(3)
    
    def GetCustomAD5(self):
        return self.GetCustomAD(4)
    
    def GetCustomAD6(self):
        return self.GetCustomAD(5)
    
    def GetCustomAD7(self):
        return self.GetCustomAD(6)
    
    def GetCustomAD8(self):
        return self.GetCustomAD(7)
    
    def GetCustomAD(self, channel):
        with self._lock:
            return self.customAD[channel]
    
    def GetCustomDIN(self):
        with self._lock:
            return self.gpio
    
    def SetCustomDOUT(self, ival):
        with self._lock:
            self.protocol.GPIO(ival)
            self.interface.sendCommand(self.protocol.createRequest())
    
    #---------------------------------- 23/29
    
    def SetMotorPolarity1(self, polarity):
        self.SetMotorPolarity(0, polarity)
    
    def SetMotorPolarity2(self, polarity):
        self.SetMotorPolarity(1, polarity)
    
    def SetMotorPolarity3(self, polarity):
        self.SetMotorPolarity(2, polarity)
    
    def SetMotorPolarity4(self, polarity):
        self.SetMotorPolarity(3, polarity)
    
    def SetMotorPolarity5(self, polarity):
        self.SetMotorPolarity(4, polarity)
    
    def SetMotorPolarity6(self, polarity):
        self.SetMotorPolarity(5, polarity)
    
    def SetMotorPolarity(self, channel, polarity):
        with self._lock:
            if polarity > 0:
                self.protocol.setPolarity(channel, SysConf.positivePolarity)
            elif polarity < 0:
                self.protocol.setPolarity(channel, SysConf.positivePolarity)
            else:
                raise ValueError('Wrong value for polarity!')
            self.interface.sendCommand(self.protocol.createRequest())
    
    @DeprecationWarning
    def EnableDCMotor(self, channel):  # TODO: decide - remove or give warning
        raise DeprecationWarning
    
    @DeprecationWarning
    def DisableDCMotor(self, channel):  # TODO: decide - remove or give warning
        raise DeprecationWarning
    
    def ResumeDCMotor(self, channel):
        with self._lock:
            self.protocol.enMotor(True, channel)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def SuspendDCMotor(self, channel):
        with self._lock:
            self.protocol.enMotor(True, channel)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def SetDcMotorPositionControlPID(self, channel, Kp, Kd, Ki_x100):  # TODO: Ki parameter
        with self._lock:
            self.protocol.setPIDParameter(SysConf.pidPositionParam, channel,
                                          SysConf.kp, Kp)
            self.interface.sendCommand(self.protocol.createRequest())
            
            self.protocol.setPIDParameter(SysConf.pidPositionParam, channel,
                                          SysConf.kd, Kd)
            self.interface.sendCommand(self.protocol.createRequest())
            
            self.protocol.setPIDParameter(SysConf.pidPositionParam, channel,
                                          SysConf.ki, Ki_x100)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def SetDcMotorVelocityControlPID(self, channel, Kp, Kd, Ki_x100):  # TODO: Ki parameter
        with self._lock:
            self.protocol.setPIDParameter(SysConf.pidVelocityParam, channel,
                                          SysConf.kp, Kp)
            self.interface.sendCommand(self.protocol.createRequest())
            
            self.protocol.setPIDParameter(SysConf.pidVelocityParam, channel,
                                          SysConf.kd, Kd)
            self.interface.sendCommand(self.protocol.createRequest())
            
            self.protocol.setPIDParameter(SysConf.pidVelocityParam, channel,
                                          SysConf.ki, Ki_x100)
            self.interface.sendCommand(self.protocol.createRequest())
    
    @DeprecationWarning
    def SetDcMotorTrajectoryPlan(self, channel, TrajPlanMthod):  # TODO:
        raise DeprecationWarning
    
    def SetDcMotorSensorFilter(self, channel, FilterMethod):  # TODO:
        raise NotImplementedError
    
    def SetDcMotorSensorUsage(self, channel, SensorType):
        with self._lock:
            self.protocol.selectMotorSensor(channel, SensorType)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def SetDcMotorControlMode(self, channel, ControlMode):
        with self._lock:
            self.protocol.selectMotorControl(channel, ControlMode)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorPositionTimeCtr(self, channel, cmdValue, timePeriod):
        with self._lock:
            self.protocol.motor(DataID.motorPos, cmdValue, channel, timePeriod)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorPositionNonTimeCtr(self, channel, cmdValue):
        with self._lock:
            self.protocol.motor(DataID.motorPos, cmdValue, channel)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorVelocityTimeCtr(self, channel, cmdValue, timePeriod):
        with self._lock:
            self.protocol.motor(DataID.motorVel, cmdValue, channel, timePeriod)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorVelocityNonTimeCtr(self, channel, cmdValue):
        with self._lock:
            self.protocol.motor(DataID.motorVel, cmdValue, channel)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorPwmTimeCtr(self, channel, cmdValue, timePeriod):
        with self._lock:
            self.protocol.motor(DataID.motorPWM, cmdValue, channel, timePeriod)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorPwmNonTimeCtr(self, channel, cmdValue):
        with self._lock:
            self.protocol.motor(DataID.motorPWM, cmdValue, channel)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorPositionTimeCtrAll(self, cmd1, cmd2, cmd3, cmd4, cmd5, cmd6,
                                  timePeriod):
        with self._lock:
            self.protocol.motor(DataID.allMotorPos,
                                [cmd1, cmd2, cmd3, cmd4, cmd5, cmd6],
                                time=timePeriod)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorPositionNonTimeCtrAll(self, cmd1, cmd2, cmd3, cmd4, cmd5, cmd6):
        with self._lock:
            self.protocol.motor(DataID.allMotorPos,
                                [cmd1, cmd2, cmd3, cmd4, cmd5, cmd6])
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorVelocityTimeCtrAll(self, cmd1, cmd2, cmd3, cmd4, cmd5, cmd6,
                                  timePeriod):
        with self._lock:
            self.protocol.motor(DataID.allMotorVel,
                                [cmd1, cmd2, cmd3, cmd4, cmd5, cmd6],
                                time=timePeriod)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorVelocityNonTimeCtrAll(self, cmd1, cmd2, cmd3, cmd4, cmd5, cmd6):
        with self._lock:
            self.protocol.motor(DataID.allMotorVel,
                                [cmd1, cmd2, cmd3, cmd4, cmd5, cmd6])
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorPwmTimeCtrAll(self, cmd1, cmd2, cmd3, cmd4, cmd5, cmd6,
                             timePeriod):
        with self._lock:
            self.protocol.motor(DataID.allMotorPWM,
                                [cmd1, cmd2, cmd3, cmd4, cmd5, cmd6],
                                time=timePeriod)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DcMotorPwmNonTimeCtrAll(self, cmd1, cmd2, cmd3, cmd4, cmd5, cmd6):
        with self._lock:
            self.protocol.motor(DataID.allMotorPWM,
                                [cmd1, cmd2, cmd3, cmd4, cmd5, cmd6])
            self.interface.sendCommand(self.protocol.createRequest())
    
    #---------------------------------- 6/7
    
    def EnableServo(self, channel):
        with self._lock:
            self.protocol.enMotor(True, channel, False)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def DisableServo(self, channel):
        with self._lock:
            self.protocol.enMotor(False, channel, False)
            self.interface.sendCommand(self.protocol.createRequest())
    
    @DeprecationWarning
    def SetServoTrajectoryPlan(self, channel, TrajPlanMthod):  # TODO:
        raise DeprecationWarning
    
    def ServoTimeCtr(self, channel, cmdValue, timePeriod):
        with self._lock:
            self.protocol.servo(DataID.servo, cmdValue, channel, timePeriod)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def ServoNonTimeCtr(self, channel, cmdValue):
        with self._lock:
            self.protocol.servo(DataID.servo, cmdValue, channel)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def ServoTimeCtrAll(self, cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, timePeriod):
        with self._lock:
            self.protocol.servo(DataID.allServo,
                                [cmd1, cmd2, cmd3, cmd4, cmd5, cmd6],
                                time=timePeriod)
            self.interface.sendCommand(self.protocol.createRequest())
    
    def ServoNonTimeCtrAll(self, cmd1, cmd2, cmd3, cmd4, cmd5, cmd6):
        with self._lock:
            self.protocol.servo(DataID.allServo,
                                [cmd1, cmd2, cmd3, cmd4, cmd5, cmd6])
            self.interface.sendCommand(self.protocol.createRequest())
    
    #---------------------------------- 1/1
    
    def LcdDisplayPMS(self, bitmap):
        bits = np.reshape(np.array(bitmap),
                     (4*self.protocol.NLCDFrames, 2*self.protocol.LCDFrameSize))
        
        chs = np.reshape(np.array(map(lambda i:
                     reduce(lambda x, y: (x<<1) + y, np.flipud(bits[i:i+8, :])),
                     range(0, bits.shape[0], 8)), dtype='uint8'),
                (self.protocol.NLCDFrames, self.protocol.LCDFrameSize))

        with self._lock:
            for i in range(Protocol.NLCDFrames):
                self.protocol.LCD(i, chs[i])
                self.interface.sendCommand(self.protocol.createRequest())
                time.sleep(0.05)
    
    def stopUpdate(self):
        with self._lock:
            self.running = False
    
    def __isRunning(self):
        with self._lock:
            return self.running
    
    def run(self):
        while self.__isRunning():
            resp = self.interface.recvPackage()
            
            with self._lock:
                self.protocol.setResponse(resp)
                
                if self.protocol.cmd == DataID.sensor:
                    # parse standard sensor package fields
                    self.us = [self.protocol.parseUS(sensor)
                                for sensor in range(self.protocol.NUS)]
                    
                    self.humanSensor = \
                        [self.protocol.parseHumanSensorAlarm(sensor)
                         for sensor in range(2)] + \
                        [self.protocol.parseHumanSensorMotion(sensor)
                         for sensor in range(2)]
                    
                    self.tilt = [self.protocol.parseTiltSensor(axis)
                                 for axis in range(2)]
                    
                    self.temp = [self.protocol.parseTemperature()] + \
                        [self.protocol.parseOverheatSensor(motor)
                          for motor in range(2)]
                    
                    self.ir = self.protocol.parseIRRange()
                    
                    self.irCode = [self.protocol.parseIRCommand(cmd)
                                   for cmd in range(4)]
                    
                    self.battery = [self.protocol.parseMBBattery(),
                                    self.protocol.parseDCBattery(),
                                    self.protocol.parseServoBattery()]
                    
                    self.voltage = [self.protocol.parseRefVoltage(),
                                    self.protocol.parsePotVoltage()]
                elif self.protocol.cmd == DataID.ADC:
                    # parse ADC package fields
                    self.customAD = [self.protocol.parseCustomAD(ch)
                                     for ch in range(self.protocol.NCUSTOMCH)]
                    self.gpio = self.protocol.parseGPIOOut()
                elif self.protocol.cmd == DataID.motorSgn:
                    # parse motor signal package fields
                    self.potentiometer = [self.protocol.parsePotentiometer(ch)
                             for ch in range(self.protocol.NMOTORS)]
                        
                    self.motorCurrent = [self.protocol.parseCurrentFeedback(ch)
                             for ch in range(self.protocol.NMOTORS)]
                    
                    self.encoderPosition = \
                        [self.protocol.parseEncoderPosition(self, enc)
                            for enc in range(2)]
                        
                    self.encoderSpeed = [self.protocol.parseEncoderSpeed(enc)
                            for enc in range(2)]
                    
                    self.encoderDirection = \
                        [self.protocol.parseEncoderDirection(self, enc)
                            for enc in range(2)]

    
    
if __name__ == '__main__':
    robot = Platform('192.168.0.201', 10001)
    
    robot.start()
    
    while False: # condition
        # TODO: implement controller
        # 1. read sensors
        # 2. compute control values
        # 3. send commands
        pass
    
    robot.stopUpdate()
    robot.join()
