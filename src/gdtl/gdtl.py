license_text='''
    Module implements API for parsing and translating a GDTL formula to an
    LTL formula. 
    Copyright (C) 2015  Cristian Ioan Vasile <cvasile@bu.edu>
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
.. module:: gdtl.py
   :synopsis: Module implements API for parsing and translating a GDTL formula
              to an LTL formula.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>

'''
'''
Created on Dec 28, 2015

@author: cristi
'''

import itertools as it

from antlr4 import CommonTokenStream, InputStream
import numpy as np
from scipy.spatial.distance import mahalanobis

from GDTLLexer import GDTLLexer
from GDTLParser import GDTLParser
from GDTLVisitor import GDTLVisitor


class Operation(object):
    '''Operation enumeration class defines the allowed Boolean and temporal
    operators used in the STL grammar.
    '''
    NOP, NOT, OR, AND, UNTIL, EVENT, ALWAYS, PRED, IMPLIES, BOOL, AP = range(11)
    binaryOPs = (OR, AND, IMPLIES, UNTIL)
    unaryOPs = (NOT, EVENT, ALWAYS)
    codes = {'!': NOT, '&&': AND, '||': OR, '=>': IMPLIES, 'U': UNTIL,
             'F': EVENT, 'G': ALWAYS}
    names = {NOT: '!', AND: '&&', OR: '||', IMPLIES: '=>', UNTIL: 'U',
             EVENT: 'F', ALWAYS: 'G'}
    
    @classmethod
    def getCode(cls, text):
        '''Gets the code corresponding to the string representation.'''
        return cls.codes.get(text, Operation.NOP)
    
    @classmethod
    def getString(cls, op):
        '''Gets custom string representation for each operation.'''
        return cls.names.get(op, 'NOP')


class Formula(object):
    '''Base class for a GDTL formula'''
    
    def __init__(self, op=Operation.NOP, **kwargs):
        '''Class constructor.'''
        self.op  = op # the operation associated with this tree node.
        if self.op in Operation.binaryOPs:
            self.left = kwargs['left']
            self.right = kwargs['right']
        elif self.op in Operation.unaryOPs:
            self.child = kwargs['child']
        elif self.op == Operation.PRED:
            self.rop = kwargs['rop'] # type of inequality predicate
            self.expr = kwargs['expression'] # lhs expression
            self.threshold = kwargs['threshold'] # value of the spatial parameter used a threshold
            self.ap = kwargs['ap']
        elif self.op == Operation.BOOL:
            self.value = bool(kwargs['value'])
    
    def formulaString(self, ltl=False):
        opname = Operation.getString(self.op)
        if self.op in Operation.binaryOPs:
            left, right = self.left.formulaString(ltl), self.right.formulaString(ltl)
            return '( {} ) {} ( {} )'.format(left, opname, right)
        elif self.op in Operation.unaryOPs:
            child = self.child.formulaString(ltl)
            return '{} ( {} )'.format(opname, child)
        elif self.op == Operation.PRED:
            if ltl:
                return self.ap
            else:
                return '({} {} {})'.format(self.expr, self.rop, self.threshold)
        elif self.op == Operation.BOOL:
            return str(self.value)
        else:
            raise ValueError('Unknown operation code %d!', self.op)
    
    def treeString(self, level=0):
        opname = Operation.getString(self.op)
        if self.op in Operation.binaryOPs:
            left, right = self.left.treeString(level+1), self.right.treeString(level+1)
            return '{offset}{op}\n{offset}left:\n{left}\n{offset}right:\n{right}'.format(
                    offset='  '*level, op=opname,left=left, right=right)
        elif self.op in Operation.unaryOPs:
            child = self.child.treeString(level+1)
            return '{offset}{op}\n{offset}child:\n{child}'.format(
                    offset='  '*level, op=opname, child=child)
        elif self.op == Operation.PRED:
            return '{}({} {} {})'.format('  '*level,
                                           self.expr, self.rop, self.threshold)
        elif self.op == Operation.BOOL:
            return str(self.value)
        else:
            raise ValueError('Unknown operation code %d!', self.op)
    
    def __str__(self):
        return self.formulaString()
    
    __repr__ = __str__


class GDTLAbstractSyntaxTreeExtractor(GDTLVisitor):
    '''Class to extract the AST of a GDTL formula from its Parse Tree.'''
    
    def __init__(self):
        super(GDTLVisitor, self).__init__()
        self.counter = it.count()
        self.ap = dict()
    
    def getAPName(self, pred):
        for k, v in self.ap.iteritems():
            if (v.rop, v.expr, v.threshold) == pred:
                return k
        return 'p{}'.format(next(self.counter))
    
    def visitFormula(self, ctx):
        op = Operation.getCode(ctx.op.text)
        if op in (Operation.OR, Operation.AND, Operation.IMPLIES, Operation.UNTIL):
            ret = Formula(op=op, left=self.visit(ctx.left), right=self.visit(ctx.right))
        elif op in (Operation.NOT, Operation.EVENT, Operation.ALWAYS):
            ret = Formula(op=op, child=self.visit(ctx.child))
        else:
            raise ValueError('Unknown operation code %d!', op)
        return ret
    
    def visitBooleanPred(self, ctx):
        return self.visit(ctx.booleanExpr())
    
    def visitBooleanExpr(self, ctx):
        opname = ctx.op.text
        if opname in ('true', 'false'):
            return Formula(op=Operation.BOOL, value=bool(opname))
        pred = Formula(op=Operation.PRED, rop=ctx.op.text,
                       expression=ctx.left.getText(),
                       threshold=float(ctx.right.getText()),
                       ap=self.getAPName((ctx.op.text, ctx.left.getText(),
                                          float(ctx.right.getText()) )) )
        self.ap[pred.ap] = pred
        return pred
    
    def visitParprop(self, ctx):
        return self.visit(ctx.child)


def gdtl2ltl(formula):
    lexer = GDTLLexer(InputStream(formula))
    tokens = CommonTokenStream(lexer)
    
    parser = GDTLParser(tokens)
    t = parser.prop()
    
    ast_parser = GDTLAbstractSyntaxTreeExtractor()
    ast = ast_parser.visit(t)
    ap = ast_parser.ap
    
    return ast, ap

mah = lambda x, P, xc: mahalanobis(x, xc, np.asarray(np.mat(P).I))
functions = {'det': np.linalg.det, 'tr': np.trace, 'mah': mah, 'norm': np.linalg.norm,
             'box': lambda x, xc, s: np.max(2*np.abs(np.asarray(x) - np.asarray(xc))/np.array(s))} 

def evalPred(pred, state, cov, state_label='x', cov_label='P', attr_dict=None):
    context = {state_label: np.array(state), cov_label: np.mat(cov)}
    context.update(functions)
    if attr_dict:
        context.update(attr_dict)
    return eval(str(pred), context)

if __name__ == '__main__':
    formula = "!(x < 10) && F y > 2 || G z<=8"
    ltl, ap = gdtl2ltl(formula)
    print ltl.formulaString(ltl=True)
    
    for pred in ['tr(P) < 2', 'tr(P) < 3', 'det(P) < 1', 'det(P)<1.01',
                 'mah(x, P, [1, 1]) < 1.41', 'mah(x, P, [1, 1]) < 1.42',
                 'theta < 2', 'theta < 2.01', 'norm(x) < 2', 'norm(x) < 3',
                 'box(x, [0.5, 1.5], [1, 1]) <= 1']:
        ltl, _ = gdtl2ltl(pred)
        print evalPred(ltl, state=[1, 2], cov=[[2, 0], [0, 0.5]], attr_dict={'theta':2})
        print
    
    formula = (("G ! ({obs}) "
                + "&& G F ( {a} && ( ! ({d2}) U {b} ) ) "
                + "&& G F ( {b} && ( ! ({d1}) U {a} ) ) "
                + "&& G {u1} "
                + "&& G (({d1} || {d2}) => {u2}) "
                + "&& G {box}").
                    format(obs = 'mah(x, P, [2.065, 1.77]) < 0.6',
                           a   = 'mah(x, P, [0.59, 0.59]) < 0.6',
                           b   = 'mah(x, P, [3.54, 2.95]) < 0.6',
                           d1  = 'mah(x, P, [2.065, 0.59]) < 0.6',
                           d2  = 'mah(x, P, [2.065, 2.95]) < 0.6',
                           u1  = 'tr(P) < 0.5',
                           u2  = 'tr(P) < 0.3',
                           box = 'box(x, [2.015, 1.77], [4.13, 3.54]) <= 1'))
    print formula
    ltl, ap = gdtl2ltl(formula)
    print ltl.treeString()
    print
    print ltl.formulaString(ltl=True)
    