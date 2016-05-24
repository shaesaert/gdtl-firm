'''
    #TODO:
    Copyright (C) 2013  Cristian Ioan Vasile <cvasile@bu.edu> and Ethan Paul Bradlow <epbradlow@gmail.com>
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
from robots import X80Pro

if __name__ == '__main__':
    robot = X80Pro('DrRobot1', '192.168.0.202', 10001) # create robot object
    robot.setSpeed([0, 0])