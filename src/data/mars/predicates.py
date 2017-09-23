#!/usr/bin/env python
license_text='''
    Mission specific predicates for the Mars scenario.
    Copyright (C) 2016  Cristian Ioan Vasile <cvasile@mit.edu>
    CSAIL, LIDS, Massachusetts Institute of Technology

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

def A(state):
    x, y, _ = state
    return ((0.2 < x < 0.6 and 3.3 < y < 3.5)
            or (1.6 < x < 2.8 and 2.0 < y < 2.3)
            or (4.1 < x < 4.5 and 0.1 < y < 0.5))

def B(state):
    x, y, _ = state
    return ((1.6 < x < 2.8 and 0.2 < y < 0.6)
            or (4.0 < x < 4.6 and 3.1 < y< 3.5))

def rock(state):
    x, y, _ = state
    return ((1.0 < x < 1.3 and 0.8 < y < 2.8)
            or (1.3 < x < 3.3 and 2.4 < y < 2.8)
            or (3.8 < x < 4.8 and 0.6 < y < 0.7))

obstacles = rock

def sand(state):
    x, y, _ = state
    return ((1.0 < x < 1.3 and 0.0 < y < 0.8)
            or (3.0 < x < 3.3 and 1.8 < y < 2.4))

def shadow(state):
    x, y, _ = state
    return x > 2.9

predicates = {
    'Obs': rock, 'A': A,'B': B, 'sand': sand, 'shadow': shadow,
}

if __name__ == '__main__':
    pass