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
    return ((0.2 < x < 0.8 and 2.8 < y < 3.4)
            or (1.6 < x < 2.8 and 1.8 < y < 2.2)
            or (4.0 < x < 4.6 and 0.2 < y < 0.8))

def existsA(state, m):
    return m.label(state, 'A')

def B(state):
    x, y, _ = state
    return ((1.6 < x < 2.8 and 0.2 < y < 0.6)
            or (3.8 < x < 4.6 and 2.8 < y< 3.4))

def existsB(state, m):
    return m.label(state, 'B')

def rock(state):
    x, y, _ = state
    return ((1.0 < x < 1.4 and 0.8 < y < 2.8)
            or (1.4 < x < 3.4 and 2.4 < y < 2.8)
            or (3.2 < x < 4.8 and 1.0 < y < 1.2))

obstacles = rock

def sand(state):
    x, y, _ = state
    return ((1.0 < x < 1.3 and 0.0 < y < 0.8)
            or (3.0 < x < 3.3 and 1.8 < y < 2.4))

def shadow(state):
    x, _, _ = state
    return x > 2.8

def done(c):
    return c.get('A', 0) >= 2 and c.get('B', 0) >= 1

predicates = {
    'Obs': rock, 'sand': sand, 'shadow': shadow,
    'A': A, 'E_A': existsA, 'B': B, 'E_B': existsB,
    'Done': done,
}

if __name__ == '__main__':
    pass