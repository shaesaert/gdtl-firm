#!/usr/bin/env python
license_text='''
    Mission specific predicates for the BU Campus scenario.
    Copyright (C) 2016  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Lab, Boston University

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
'''
Created on May 26, 2016

@author: cristi
'''

def obstacles(state):
    x, y = state
    return ((0 < x < 4.88 and 2.37 < y < 3.64)
            or (1 < y < 1.54 and (0 < x < 0.59
                                  or 1.32 < x < 2.65
                                  or 3.42 < x < 3.69
                                  or 4.25 < x < 4.88
                                  )
               )
           )

def bridge1(state):
    x, y = state
    return 1 < y < 1.54 and (2.65 < x < 3.42 or 3.69 < x < 4.25)

def bridge2(state):
    x, y = state
    return 1 < y < 1.54 and 0.59 < x < 1.32

def march_plaza(state):
    x, y = state
    return 1.79 < y < 2.28 and 0.70 < x < 1.27

def kenmore_square(state):
    x, y = state
    return 1.57 < y < 2.07 and 3.78 < x < 4.46

def audubon_circle(state):
    x, y = state
    return 0.12 < y < 0.55 and 1.46 < x < 1.92

def fenway_stadium(state):
    x, y = state
    return 0.40 < y < 0.95 and 3.68 < x < 4.88

predicates = {
    'Obs': obstacles, 'B1': bridge1, 'B2': bridge2, 'M': march_plaza,
    'K': kenmore_square, 'A': audubon_circle,'S': fenway_stadium
}

if __name__ == '__main__':
    pass