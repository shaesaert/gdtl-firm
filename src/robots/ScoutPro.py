'''
    High level module for interacting with DrRobot ScoutPro robot.
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

from numpy import array,pi

from DrRobot import Platform


class ScoutPro(Platform):
    ''' A high level DrRobot ScoutPro interface.'''
    
    # robot's structural parameters
#     wheelsDistance = 0.295      # distance between the wheels [m]
    wheelDiameter = 2 * 0.0825  # wheel radius [m]
    pulsesPerRevolution = 756   # number of pulses per wheel revolution
#     pulse = pulsesPerRevolution / (pi * wheelDiameter) # 1 m [pulse] = N/(pi*wheelDiameter)
    pulse = 756 / (pi * 295) # 1 m [pulse] = N/(pi*wheelDiameter); N=752 #TODO:
    wheelsDistance = 295 # distance between the wheels [m]; #TODO:
    
    NoIR = 7
    NoUS = 6
    maxSpeed = 1500 # maximum speed [pulses]
    minSpeed = 300  # minimum speed [pulses] 
    
    
    def __init__(self, name, hostname, portno=10001):
        '''
        Constructor
        '''
        Platform.__init__(self, hostname, portno)
        
        # name of the robot
        self.name = name

        # TODO: initialize local state
        
        # TODO: initialize robot's hardware configuration
        # 1. enable motors
        # 2. enable sending sensor readings
        # 3. set encoders/potentiometers (single/dual potentiometer(s) or encoder)
        # 4. set motor control (direct PWM, position, velocity)
        
        # TODO: initialize robot state update thread
        # TODO: initialize locks on robot state
        
    def __str__(self):
        return self.name
    
    #---------------------------------------------------------------------------
    @classmethod
    def ad2ir(cls, ad_value):
        '''Transforms a raw IR value from the AD to a distance in meters.'''
        if (ad_value > 4095) or (ad_value <= 0):
            return 0
        distance = 0.216 /(ad_value * 3.0 / 4028 - 0.17)
        return max(min(distance, 0.81), 0.09)
    
    @classmethod
    def vel2pulse(cls, vel):
        '''Transforms the velocity [m/s] in encoder speed [pulse/s].'''
        return int(max(min(vel*cls.pulse, cls.maxSpeed), -cls.maxSpeed))
    
    @classmethod
    def pulse2vel(cls, vpulse):
        '''Transforms the velocity [m/s] in encoder speed [pulse/s].'''
        return vpulse/cls.pulse
    
    #---------------------------------------------------------------------------
    def getIR(self):
        '''Returns IR distance sensors' measurements in meters.'''
        with self._lock:
            return array(map(ScoutPro.ad2ir, [self.ir] + self.customAD[2:]))
    
    def getUS(self, nr):
        '''Return US distance sensors' measurements in meters.'''
        with self._lock:
            return array(self.us)/100.0
    
    def setVelocity(self, linear, angular):
        '''Sets the speed of the two motors.'''
        self.setSpeed([linear - (ScoutPro.wheelsDistance*angular)/2,  # left
                       linear + (ScoutPro.wheelsDistance*angular)/2]) # right
#         print 'Final Angle Control: ' , (X80Pro.wheelsDistance*angular)/2
    
    def setSpeed(self, speed, timePeriod=None):
        '''Sets the speed of the two motors.'''
        if len(speed) != 2:
            raise ValueError('invalid number of values in list')
        sLeft = ScoutPro.vel2pulse(speed[0])
        #print 'speed[0]: ', speed[0] 
        sRight = ScoutPro.vel2pulse(speed[1])
        #print 'speed[1]: ', speed[1] 
        if timePeriod:
            self.DcMotorPositionTimeCtrAll(sLeft,-sRight, 0, 0, 0, 0, timePeriod)
        else:
#             print 'Send cmd:', -sLeft, sRight
            self.DcMotorVelocityNonTimeCtrAll(sLeft,-sRight, 0, 0, 0, 0)
        
    def getEncoderPositions(self):
        '''Returns the encoder values for the two motors.'''
        #TODO: check encoder direction
        with self._lock:
            return array(self.encoderPosition)*array(self.encoderDirection)
        
    def getEncoderSpeeds(self):
        '''Returns the encoder speed values for the two motors.'''
        #TODO: check encoder direction
        with self._lock:
            return array(self.encoderSpeed)*array(self.encoderDirection)
    
    #---------------------------------------------------------------------------
    def __configurePosPID(self, nr, Kp, Ki, Kd):
        raise NotImplementedError #TODO:
#        if self.interface.sendCommand('H', nr, Kp, Ki, Kd):
#            self.cont[nr]['Kp'] = Kp
#            self.cont[nr]['Ki'] = Ki
#            self.cont[nr]['Kd'] = Kd
#            return True
#        return False
    
    def __configureVelPID(self, nr, Kp, Ki, Kd):
        raise NotImplementedError #TODO:
#        if self.interface.sendCommand('H', nr, Kp, Ki, Kd):
#            self.cont[nr]['Kp'] = Kp
#            self.cont[nr]['Ki'] = Ki
#            self.cont[nr]['Kd'] = Kd
#            return True
#        return False
    
    def __getMotorCurrent(self):
        ''' #TODO:
        MotorCurrent[0] = val1 / 728.0
        MotorCurrent[1] = val2 / 728.0
        '''
        raise NotImplementedError
    
    def __getBattery(self):
        '''
        BoardVol = value / 4095.0 * 9.0;
        DCMotorVol = value / 4095.0 * 24.0
        '''
        raise NotImplementedError #TODO:
    
    #---------------------------------------------------------------------------
    
    def forward(self, distance):
        raise NotImplementedError
        '''
        private void runStep(double runDis)
            //the robot will go forward the rundistance
            int diffEncoder = (int)( (runDis / ( 2 * Math.PI * WheelR ) ) * CircleCnt);
            
            int LeftTarget = sock.EncoderPos[0] - diffEncoder;
            if(LeftTarget < 0){
                LeftTarget = 32767 + LeftTarget;
            }
            else if(LeftTarget > 32767 ){
                LeftTarget = LeftTarget - 32767;
            }
            
            int RightTarget = sock.EncoderPos[1] + diffEncoder;
            if(RightTarget > 32767){
                RightTarget =RightTarget - 32767;
            }
            else if(RightTarget < 0){
                RightTarget = 32767 + RightTarget;
            }
            
            
            SetDcMotorControlMode( (byte)0, POSITIONCTRL);
            SetDcMotorControlMode( (byte)1, POSITIONCTRL);
            SetDcMotorPositionControlPID((byte)0,1000,30,2000);
            SetDcMotorPositionControlPID((byte)1,1000,30,2000);
            
            DCMotorPositionTimeCtrlAll(LeftTarget,RightTarget,NONCTRLCMD,NONCTRLCMD,NONCTRLCMD,NONCTRLCMD,10000);
        '''

    def turn(self, angle):
        raise NotImplementedError
        '''
        private void turnStep(double angle)
            //the parameter is degree, need transfor to radian
            angle = angle / 180 * Math.PI;
            int diffEncoder = (int)(( ( angle * WheelDis / 2) / (2 * Math.PI * WheelR)) * CircleCnt );
            int LeftTarget = sock.EncoderPos[0] + diffEncoder;
            if(LeftTarget < 0){
                LeftTarget = 32767 + LeftTarget;
            }
            else if(LeftTarget > 32767 ){
                LeftTarget = LeftTarget - 32767;
            }
            
            int RightTarget = sock.EncoderPos[1] + diffEncoder;
            if(RightTarget > 32767){
                RightTarget =RightTarget - 32767;
            }
            else if(RightTarget < 0){
                RightTarget = 32767 + RightTarget;
            }
            
            SetDcMotorControlMode( (byte)0, POSITIONCTRL);
            SetDcMotorControlMode( (byte)1, POSITIONCTRL);
            
            DCMotorPositionTimeCtrlAll(LeftTarget,RightTarget,NONCTRLCMD,NONCTRLCMD,NONCTRLCMD,NONCTRLCMD,5000);
        '''
