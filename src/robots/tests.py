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

from comm import DataID
from comm import Protocol
from comm import UDPIPInterface
from comm import _checksum

from DrRobot import Platform

#-----------------------------------TESTS---------------------------------------
def testConnection():
    print 'Connecting ...'
    conn = UDPIPInterface('192.168.0.220', 10001)
    
    conn.connect()
    
    print 'Connected'
    
    data = (Protocol.ridPMS5005, Protocol.rvs, DataID.commSys, 1, 1)
    print 'send command'
    conn.sendCommand(bytearray(
                               Protocol.STX + data + \
                               (_checksum(data),) +\
                               Protocol.EXT                               
                               )
                    )
    print 'command sent'
    
    for i in range(100):
        package = conn.recvPackage()
        print 'Protocol', i, '->', ' '.join(map(lambda x: str(int(x)), bytearray(package)))
    
    conn.disconnect()
    print 'Disconnected'

def testProtocolAPI():
    print 'Connecting ...'
    conn = UDPIPInterface('192.168.0.220', 10002)
    conn.connect()
    print 'Connected'
    
    package = Protocol()
    package.commSys()
    print 'send command'
    conn.sendCommand(package.createRequest())
    print 'command sent'
    
    for i in range(10):
        resp = conn.recvPackage()
        print 'Protocol', i, '->', ' '.join(map(lambda x: str(x).rjust(2), bytearray(resp)))
        package.setResponse(resp)
        print 'CMD:', package.cmd
        
        if package.cmd == DataID.sensor:
            print 'Sensor package'
            print [package.parseUS(i) for i in range(6)]
        elif package.cmd == DataID.ADC:
            print 'ADC package'
            print [package.parseCustomAD(i) for i in range(8)]
        elif package.cmd == DataID.motorSgn:
            print 'Motor package'
            print [package.parseEncoderPosition(i) for i in range(2)]
            print [package.parseEncoderSpeed(i) for i in range(2)]
    
    conn.disconnect()
    print 'Disconnected'

def testMovement(): # TODO: recode to test new implementation
    import time
    
    print 'Connecting ...'
    conn = UDPIPInterface('192.168.0.202', 10001)
    conn.connect()
    print 'Connected'
    
    package = Protocol()
    
    # Move speed forward
    package.motor(DataID.allMotorVel, (800, 800, 0, 0, 0, 0))
    conn.sendCommand(package.createRequest())
    time.sleep(3)
    
    package.motor(DataID.allMotorVel, (0, 0, 0, 0, 0, 0))
    conn.sendCommand(package.createRequest())    
    time.sleep(2)
    return
    
    #Move speed backward
    package.motor(DataID.allMotorVel, (500, -500, 0, 0, 0, 0))
    conn.sendCommand(package.createRequest())
    time.sleep(5)
    
    package.motor(DataID.allMotorVel, (0, 0, 0, 0, 0, 0))
    conn.sendCommand(package.createRequest())
    time.sleep(2)
    
    # Move speed forward
    package.motor(DataID.allMotorVel, -500, 0)
    conn.sendCommand(package.createRequest())
    package.motor(DataID.allMotorVel, 500, 1)
    conn.sendCommand(package.createRequest())
    time.sleep(5)
    
    package.motor(DataID.allMotorVel, 0, 0)
    conn.sendCommand(package.createRequest())
    package.motor(DataID.allMotorVel, 0, 1)
    conn.sendCommand(package.createRequest())
    time.sleep(2)
    
    #Move speed backward
    package.motor(DataID.allMotorVel, 500, 0)
    conn.sendCommand(package.createRequest())
    package.motor(DataID.allMotorVel, -500, 1)
    conn.sendCommand(package.createRequest())
    time.sleep(5)
    
    package.motor(DataID.allMotorVel, 0, 0)
    conn.sendCommand(package.createRequest())
    package.motor(DataID.allMotorVel, 0, 1)
    conn.sendCommand(package.createRequest())
    
    conn.disconnect()
    print 'Disconnected'
    
def testServo(): # TODO: recode to test new implementation
    import time
    
    print 'Connecting ...'
    conn = UDPIPInterface('192.168.0.201', 10001)
    conn.connect()
    print 'Connected'
    
    package = Protocol()
    
    # Enable servos
    package.enMotor()
    conn.sendCommand(package.createRequest())
    time.sleep(5)
    
    print 'Set servo position to (4500, 4500, 4500)'
    package.servo(DataID.allServo, (3000, )*6)
    conn.sendCommand(package.createRequest())
    time.sleep(5)
    
    print 'Set servo position to (3000, 3000, 3000)'
    package.servo(DataID.allServo, (4500, )*6)
    conn.sendCommand(package.createRequest())
    time.sleep(3)
    
    for i in range(Protocol.NSERVOS):
        package.enMotor(False, i, False)
        conn.sendCommand(package.createRequest())
    
    conn.disconnect()
    print 'Disconnected'

def testCRC():
    '''
    Tests for the CRC checksum computation (examples taken from documentation).
    '''
    from comm import _checksum
    
    print '------------------------------Test CRC------------------------------'
    comp = (_checksum((Protocol.ridHost, Protocol.rvs, DataID.commSys, 1, 1)),
            _checksum((Protocol.ridPMS5005, Protocol.rvs, 28, 3, 2, 0, 8)),
            _checksum((Protocol.ridPMS5005, Protocol.rvs, 30, 2, 0, 7)),
            _checksum((Protocol.ridPMS5005, Protocol.rvs, 5, 3, 4, 160, 15)),
            _checksum((Protocol.ridPMS5005, Protocol.rvs, 7, 3, 13, 0, 2)),
            _checksum((Protocol.ridPMS5005, Protocol.rvs, 7, 3, 14, 0, 1)),
            _checksum((Protocol.ridPMS5005, Protocol.rvs, 3, 3, 0, 112, 23)))
    actual = (72, 101, 214, 86, 72, 78, 83)
    
    for (c, a) in zip(comp, actual):
        if c == a:
            print "[PASS]",
        else:
            print "[FAIL]",
        print 'Computed CRC:', c, 'Actual CRC:', a


def testLCD():
    '''
    Test for the LCD
    '''
    import time
    
    print 'Connecting ...'
    conn = UDPIPInterface('192.168.0.202', 10001)
    conn.connect()
    print 'Connected'
    
#    chs = bytearray([0x05] + [0]*(128*8-1))
    with open('gryph_th.pbm', 'r') as f:
#     with open('test.pbm', 'r') as f:
        chs = bytearray()
        bits = ''.join(map(lambda x: x.strip(), f.readlines())).replace(' ', '')
        
        
        for j in range(8):
            for i in range(128):
                byte = ''
                for k in range(8):
                    byte = bits[i + (j*8+k)*128] + byte
                chs.append(int(byte, 2))
        
#        print len(bits), 128*64, len(chs), 128*64/8
#        print bits
#        print map(hex, chs)
        print map(str, chs)
    
    package = Protocol()
    for i in range(Protocol.NLCDFrames):
        package.LCD(i, chs[i*64:(i+1)*64])
        print 'send command'
        conn.sendCommand(package.createRequest())
        print 'command sent'
        time.sleep(0.05)
        
    conn.disconnect()
    print 'Disconnected'

def testMovPackage():
    package = Protocol()
    package.motor(DataID.allMotorVel, (0, 0, -32768, -32768, -32768, -32768))
    print map(hex, bytearray(package.createRequest()))
    
def stop():
    Platform.DcMotorVelocityNonTimeCtrAll(0, 0, 0, 0, 0, 0)
    print 'HALT'

if __name__ == '__main__':
#    testCRC()
#     testConnection()
#     testProtocolAPI()
    testMovement()
#    testServo()
#     testLCD()
#     testMovPackage()
#     stop()
    pass
