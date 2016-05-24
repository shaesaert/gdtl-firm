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

import sys
import socket
import logging


# logging for processed packages
logfilename = 'updip.log'
logformat = '%(asctime)s %(name)s %(levelname)s %(message)s'
loglevel = logging.DEBUG

logger = logging.getLogger(__name__)
logfile = logging.FileHandler(logfilename)
formatter = logging.Formatter(logformat)
logfile.setFormatter(formatter)
logger.addHandler(logfile)
logger.addHandler(logging.StreamHandler(sys.stdout))
logger.setLevel(loglevel)


def _checksum(data):
    '''Compute data checksum.'''
    shiftReg = 0
    
    for b in data:
        for _ in range(8):
            if (b ^ shiftReg) & 0x01:
                shiftReg = (shiftReg >> 1) ^ 0x8C
            else:
                shiftReg = (shiftReg >> 1)
            b = b >> 1
    return shiftReg

def toStr(a, delim='|'): #TODO: comment or delete
    return delim.join(map(lambda x: str(x).rjust(3), bytearray(a)))

def toHex(a, delim=' '): #TODO: comment or delete
    return delim.join(map(lambda x: hex(x)[2:], bytearray(a)))


class UDPIPInterface(object):
    '''
    UDP/IP interface for communication with a Robot. It implements low level
    package communication with a robot.
    '''
    
    def __init__(self, host, port, bufferSize=2**14):
        '''
        Constructor - initializes the interface with the host name, port number
        and buffersize (default= 2**14 = 16384 bytes).
        '''
        self.port = port
        self.host = host
        self.__connected = False
        self.bufferSize = bufferSize
    
    def connect(self):
        '''
        Initializes a new socket with the stored host name and port number.
        '''
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__sock.connect((self.host, self.port))
        self.__connected = True
        
        logger.info('Connected to %s on port %s!', self.host, self.port)
    
    def disconnect(self):
        '''
        Closes the current socket.
        '''
        if self.isConnected():
            self.__sock.close()
            self.__connected = False
            logger.info('Disconnected from %s!', self.host)
        else:
            logger.debug('Attempt to disconnect while not connected!')
    
    def isConnected(self):
        '''
        Tests if the socket is initialized.
        '''
        return self.__connected
    
    def __del__(self):
        '''
        Destructor - closes the socket.
        '''
        if self.isConnected():
            self.disconnect()
    
    def sendCommand(self, cmd):
        '''
        Sends the given command.
        '''
        # check if message is empty
        if not cmd:
            return False
                
        # send message
        self.__sock.sendall(cmd)
        logger.debug('command sent: %s', toHex(cmd))
            
    def recvPackage(self):
        '''
        Reads a package from the host, at most self.bufferSize bytes.
        '''
        resp = self.__sock.recv(self.bufferSize)
        logger.debug('Package received: %s', toHex(resp))
        
        return resp


class DataID(object):
    '''
    Data IDs structure used by the robot's firmware.
    '''
    motorPos      = 3
    allMotorPos   = 4
    motorPWM      = 5
    allMotorPWM   = 6
    system        = 7
    GPIO          = 22
    LCD           = 23
    motorVel      = 26
    allMotorVel   = 27
    servo         = 28
    allServo      = 29
    enMotors      = 30
    motorSgn      = 123
    ADC           = 124
    sensor        = 125
    allSensor     = 127
    commSys       = 255
    
    cmds = (motorPos, allMotorPos, motorPWM, allMotorPWM, system, GPIO, LCD, \
            motorVel, allMotorVel, servo, allServo, enMotors, motorSgn, \
            ADC, sensor, allSensor, commSys)

    motorCmds = (motorPos, motorPWM, motorVel,
                 allMotorPos, allMotorPWM, allMotorVel)
    
    servoCmds = (servo, allServo)
    
    reqSensorData = (motorSgn, ADC, sensor, allSensor)
    
    

class SysConf(object):
    '''
    TODO: add description
    '''
    
    # sub-commands
    motorPolarity    = 0x06
    pidPositionParam = 0x07
    pidVelocityParam = 0x06
    motorSensor      = 0x0D
    motorControl     = 0x0E
    collisionProtect = 0x13 # Note: DRK6000/8000 use only
    
    # options
    positivePolarity = 0x01
    negativePolarity = 0xFF
    polarityValues = (positivePolarity, negativePolarity)
    
    # PID parameters
    kp = 0x01
    kd = 0x02
    ki = 0x03
    
    # potentiometer options
    singlePotentiometer = 0x00
    dualPotentiometer   = 0x01
    encoder             = 0x02
    sensorTypes = (singlePotentiometer, dualPotentiometer, encoder) 
    
    # motor control options
    pwmControl      = 0x00
    positionControl = 0x01
    velocityControl = 0x02
    controlTypes = (pwmControl, positionControl, velocityControl)
    
    
    pidParamTypes = (pidPositionParam, pidVelocityParam)
    pidParam = (kp, kd, ki)
    



class Protocol(object):
    '''
    Package handling class. It implements DrRobot communication protocol.
    '''
    
    STX = (0x5E, 0x02) # start label
    EXT = (0x5E, 0x0D) # stop label
    FLAG = 28 # delimiter flag in some packages
    
    NMOTORS   = 6 # number of DC motors
    NSERVOS   = 6 # number of servos
    NUS       = 6 # number of US sensors
    NHS       = 2 # number of Human Sensors
    NCUSTOMCH = 8 # number of custom AD channels
    NLCDFrames   = 16 # number of LCD frames
    LCDFrameSize = 64 # LCD frame size in bytes
    
    
    # Robot ID: Host(PC) - 0, PMS5005 - 1, PMB5010 - 8
    ridHost    = 0x00
    ridPMS5005 = 0x01
    ridPMB5010 = 0x10
    
    # Reserved byte
    rvs = 0x00
    
    # packages
    ack = bytearray(STX + (ridHost, rvs, DataID.commSys, 0x01, 0x01, 72) + EXT)

    # Ping codes
    PING_PMS5005     = 0x01
    RVS_PING_PMS5005 = 0x00

    
    def __init__(self, bufferSize=4096):
        self.cmd = None
        self.rid = None
        self.data = None
        self.bufferSize = bufferSize
        
    def createRequest(self): #TODO: add logging
        if self.cmd is None:
            return False
        if self.rid is None:
            return False
        if self.data is None:
            return False
        
        msg = bytearray(Protocol.STX) # put start of transmission indicator
        msg.append(self.rid) # append robot ID (PMS5005 or PMS5010)
        msg.append(Protocol.rvs) # append the reserved byte
        msg.append(self.cmd) # append command
        msg.append(len(self.data) & 0xFF) # append package length
        msg.extend(self.data) # append actual data
        msg.append(_checksum(msg[2:])) # compute and append checksum
        msg.extend(Protocol.EXT) # append end of transmission indicator
        
        return str(msg)
    
    #---------------------------------------------------------------------------
    
    def motor(self, cmd, data, ch=-1, time=None):
        if cmd not in DataID.motorCmds:
            raise ValueError('Wrong command for motors!')
        
        self.cmd = cmd
        self.rid = Protocol.ridPMS5005
        
        self.data = bytearray()
        
        if 0 <= ch < Protocol.NMOTORS:
            self.data.append(ch) # append motor channel
            # append motor target value
            self.data.append(data & 0x00FF) # low byte
            self.data.append((data >> 8) & 0x00FF) # high byte
        else:
            for ch in range(Protocol.NMOTORS): # append value for each motor
                self.data.append(data[ch] & 0x00FF) # low byte
                self.data.append((data[ch] >> 8) & 0x00FF) # high byte
        
        if time != None:
            self.data.append(Protocol.FLAG) # append flag
            # append time
            self.data.append(time & 0x00FF) # low byte
            self.data.append((time >> 8) & 0x00FF) # high byte    
    
    def servo(self, cmd, position, ch=-1, time=None):
        if cmd not in DataID.servoCmds:
            raise ValueError('Wrong command for servos!')
        
        self.cmd = cmd
        self.rid = Protocol.ridPMS5005
        
        self.data = bytearray()
        
        
        if 0 <= ch < Protocol.NSERVOS:
            self.data.append(ch) # append servo channel
            # append servo target position
            self.data.append(position & 0x00FF) # low byte
            self.data.append((position >> 8) & 0x00FF) # high byte
        else:
            for ch in range(Protocol.NSERVOS): # append position for each servo
                self.data.append(position[ch] & 0x00FF) # low byte
                self.data.append((position[ch] & 0xFF00) >> 8) # high byte
        
        if time != None:
            # append flag
            self.data.append(Protocol.FLAG)
            
            # append time
            self.data.append(time & 0x00FF) # low byte
            self.data.append((time >> 8) & 0x00FF) # high byte
    
    def enMotor(self, en=True, ch=-1, isDC=True):
        self.cmd = DataID.enMotors
        self.rid = Protocol.ridPMS5005
        
        msg = en & 0x01
        if ch in range(Protocol.NMOTORS) and isDC:
            msg = msg + ch
        elif ch in range(Protocol.NSERVOS) and (not isDC):
            msg = msg + Protocol.NMOTORS + ch
        self.data = bytearray((msg,))
    
    def GPIO(self, gpio):
        self.cmd = DataID.GPIO
        self.rid = Protocol.ridPMS5005
        
        msg = 0
        for i in range(len(gpio)-1, -1, -1):
            msg = msg <<1 | 0x01&gpio[i]
        self.data = bytearray((msg & 0xFF,))
    
    def requestSensorData(self, cmd, number=-1):
        if cmd not in DataID.reqSensorData:
            raise ValueError('Wrong command for sensor data request!')
        
        self.cmd = cmd
        self.rid = Protocol.ridPMS5005
        
        self.data = bytearray()
        if number >= 0:
            self.data.append(number & 0xFF)
    
    def LCD(self, frame, rows):
        self.cmd = DataID.LCD
        self.rid = Protocol.ridPMS5005
        
        if not (0 <= frame <= Protocol.NLCDFrames):
            raise ValueError('Wrong frame number!')
        self.data = bytearray([frame])
        
        if len(rows) != Protocol.LCDFrameSize: 
            raise ValueError('Wrong frame size!')
        self.data.extend(rows)

    def setPolarity(self, ch, polarity=SysConf.positivePolarity):
        if not (0 <= ch < Protocol.NMOTORS): # check motor channel
            raise ValueError('Invalid motor channel (%d)!', ch)
        if polarity not in SysConf.polarityValues: # check polarity value
            raise ValueError('Wrong polarity value!')
        self.system(SysConf.motorPolarity, [ch, polarity])
    
    def setPIDParameter(self, control, ch, param, value):
        if control not in SysConf.pidParamTypes:
            raise ValueError('Unknown control type!')
        if not (0 <= ch < Protocol.NMOTORS): # check motor channel
            raise ValueError('Invalid motor channel (%d)!', ch)
        if param not in SysConf.pidParam:
            raise ValueError('Unknown parameter!')
        
        if param != SysConf.ki:
            self.system(SysConf.pidPositionParam,
                             [ch, param, value & 0x00FF, (value & 0xFF00) >> 8])
        self.system(SysConf.pidPositionParam, [ch, param, #TODO: check correctness
                           int(value) & 0x00FF, (int(1.0/value) & 0xFF00) >> 8])
    
    def selectMotorSensor(self, ch, sensor=SysConf.singlePotentiometer):
        if not (0 <= ch < Protocol.NMOTORS): # check motor channel
            raise ValueError('Invalid motor channel (%d)!', ch)
        if sensor not in SysConf.sensorTypes: # check sensor type
            raise ValueError('Unknown sensor type!')
        self.system(SysConf.motorSensor, [ch, sensor])
    
    def selectMotorControl(self, ch, control=SysConf.velocityControl):
        if not (0 <= ch < Protocol.NMOTORS): # check motor channel
            raise ValueError('Invalid motor channel (%d)!', ch)
        if control not in SysConf.controlTypes: # check control type
            raise ValueError('Unknown control type!')
        self.system(SysConf.motorSensor, [ch, control])
    
    def enableCollisionProtection(self, protect=False):
        self.system(SysConf.collisionProtect, [int(protect) & 0x01])

    def system(self, subcmd, params):
        self.cmd = DataID.system
        self.rid = Protocol.ridPMS5005
        
        self.data = bytearray([subcmd & 0xFF])
        self.data.extend(params)
    
    def commSys(self):
        self.cmd = DataID.commSys
        self.rid = Protocol.ridPMS5005
        self.data = bytearray(self.PING_PMS5005)

    #--------------------------------Response Package---------------------------
    def setResponse(self, msg):
        # check a message if it is consistent i.e. has the specified format
        # STX - ridHost - rsv - cmd - length - data - checksum - EXT
        # and parse response
        msg = bytearray(msg)
#        logger.debug('Message: %s', toHex(msg))
        if tuple(msg[:2]) != Protocol.STX:
            logger.debug('Start indicator not present! Got %s instead!', toStr(msg[:2]))
            return False
        if msg[2] != Protocol.ridHost:
            logger.debug('RID id different that Host RID! Got %s instead!', toStr(msg[2:3]))
            return False
        
        length = msg[5]
        
        print 'Package length:', len(msg), msg[5]
        print 'Package:', toStr(msg)
        
        # FIXME: problem when getting more than one package per recv call
#        if len(msg) != length + 9:
#            logger.debug('Wrong message length!')
#            return False
        
        crc = _checksum(msg[2:-3])
        if crc != msg[-3]:
            logger.debug('Checksum does not match! Got %d instead!', crc)
            return False
        
        self.cmd = msg[4]
        self.rid = Protocol.ridHost
        self.data = msg[6:-3]
        
        logger.debug('Set cmd=%d rid=%d data=%s!', self.cmd, self.rid, toHex(self.data))
        
        return True
    
    def parseField(self, offset, mask1=0xFF, mask2=0x00):
        logger.debug('Parse [offset=%d]: high byte[%s] high mask[%s] low byte[%s] low mask[%s] result[%d][%s]',
                     offset, hex(self.data[offset+1]),
                     hex(mask2),
                     hex(self.data[offset]),
                     hex(mask1),
                     (self.data[offset+1] & mask2) << 8 | self.data[offset] & mask1,
                     hex((self.data[offset+1] & mask2) << 8 | self.data[offset] & mask1)
                     )
        return (self.data[offset+1] & mask2) << 8 | self.data[offset] & mask1

    #--------------------------------Motor Package------------------------------
    def parsePotentiometer(self, channel): # TODO: replace with named constant
        if self.cmd != DataID.motorSgn: return False
        return self.parseField(2*channel, 0xFF, 0x7F)
    
    def parseCurrentFeedback(self, channel): # TODO: replace with named constant
        if self.cmd != DataID.motorSgn: return False
        return self.parseField(2*Protocol.NMOTORS + 2*channel, 0xFF, 0x7F)
    
    def parseEncoderPosition(self, enc): # TODO: replace with named constant
        if self.cmd != DataID.motorSgn: return False
        return self.parseField(4*Protocol.NMOTORS + 4*enc, 0xFF, 0xFF)
    
    def parseEncoderSpeed(self, enc): # TODO: replace with named constant
        if self.cmd != DataID.motorSgn: return False
        return self.parseField(4*Protocol.NMOTORS + 4*enc + 2, 0xFF, 0xFF)
    
    def parseEncoderDirection(self, enc): # TODO: replace with named constant
        if self.cmd != DataID.motorSgn: return False
        return self.parseField(4*Protocol.NMOTORS + 8, 1<<enc)

    #-------------------------------Sensor Package------------------------------
    def parseUS(self, sensor): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(sensor)
    
    def parseHumanSensorAlarm(self, sensor): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*sensor, 0xFF, 0x0F)
    
    def parseHumanSensorMotion(self, sensor): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*sensor + 2, 0xFF, 0x0F)
    
    def parseTiltSensor(self, axis): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 2*axis, 0xFF, 0x0F)
    
    def parseOverheatSensor(self, motor): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 4 + motor*2, 0xFF, 0x0F)
    
    def parseTemperature(self): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 8, 0xFF, 0x0F)
    
    def parseIRRange(self): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 10, 0xFF, 0x0F)
    
    def parseIRCommand(self, cmd): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 12 + cmd)
    
    def parseMBBattery(self): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 16, 0xFF, 0x0F)
    
    def parseDCBattery(self): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 18, 0xFF, 0x0F)
    
    def parseServoBattery(self): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 20, 0xFF, 0x0F)
    
    def parseRefVoltage(self): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 22, 0xFF, 0x0F)
    
    def parsePotVoltage(self): # TODO: replace with named constant
        if self.cmd != DataID.sensor: return False
        return self.parseField(self.NUS + 4*self.NHS + 24, 0xFF, 0x0F)
    
    #-------------------------------Custom Package------------------------------
    def parseCustomAD(self, channel): # TODO: replace with named constant
        if self.cmd != DataID.ADC: return False
        return self.parseField(2*channel, 0x0FF, 0x0F)
    
    def parseGPIOOut(self): # TODO: replace with named constant
        if self.cmd != DataID.ADC: return False
        return self.parseField(2*self.NCUSTOMCH)
    
    #---------------------------------Ack Package-------------------------------
    def isAck(self, msg): #FIXME: received reserve byte may not be 0
        return self.ack == msg
