#!/usr/bin/env python
license_text='''
    Mission specific predicates for the Mars scenario.
    Copyright (C) 2017  Cristian Ioan Vasile <cvasile@mit.edu>
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

'''
Note: Using predicates to define the regions of interest has the advantage that
it is independent of the representation of the map.
'''

def priorA(state):
    x, y, _ = state
    return ((-7.5 < x < -4.5 and -13 < y < -7)
            or (6 < x < 12 and -15 < y < -13))

def A(state, m):
    return m.value(state, 'A')

def priorB(state):
    x, y, _ = state
    return ((3 < x < 7 and 6 < y < 10))

def B(state, m):
    return m.value(state, 'B')

def rock(state):
    x, y, _ = state
    return ((6 < x < 12 and -10 < y < -4)
            or (-15 < x < -11 and -15 < y < -11))

def rock_later(state):
    x, y, _ = state
    return (-7 < x < -1 and 3 < y < 9)

def sand(state):
    x, y, _ = state
    return ((1.0 < x < 1.3 and 0.0 < y < 0.8)
            or (3.0 < x < 3.3 and 1.8 < y < 2.4))

def shadow(state):
    '''Predicate defining the visible region of the map.'''
    x, y, _ = state
#     return x > 2.8
    return (x > -5 or y > 0)

# def done(c):
#     '''Predicate defining the end of the mission. It is used to convert DTL
#     to scDTL and optimize specification automaton.
#     '''
#     return c.get('A', 0) >= 2 and c.get('B', 0) >= 1

# dictionary containing all predicates used in the specification
predicates = {
    'Obs': rock, 'sand': sand, 'shadow': shadow,
    'A': A, 'B': B,
    'ObsLater': rock_later,
#     'Done': done,
}

# predicate defining obstacles; it is used to optimize sampling
obstacles = rock

# dictionary defining the layers of the map with associate priors 
layer_priors = {
    # NOTE: priors are for now deterministic and given by the underlying
    # predicate values
    'Obs': rock, 'ObsLater': rock_later, 'sand': sand, 'shadow': shadow, 'A': priorA, 'B': priorB
}
# set of collectable items; must be a subset of the keys set of layer_priors 
collectables = {'A', 'B'}

if __name__ == '__main__':
    pass
