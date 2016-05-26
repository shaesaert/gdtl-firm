grammar GDTL;

prop:
         '(' child=prop ')' #parprop
    |    booleanExpr #booleanPred
    |    op='!' child=prop #formula
    |    op='F' child=prop #formula
    |    op='G' child=prop #formula
    |    left=prop op='=>' right=prop #formula
    |    left=prop op='&&' right=prop #formula
    |    left=prop op='||' right=prop #formula
    |    left=prop op='U' right=prop #formula
    ;
expr:
        ('-('|'(') expr ')'
    |   expr '^' expr
    |   ('tr('|'det('|'norm(') expr ')'
    |   'mah(' expr ',' expr ',' expr ')'
    |   'box(' expr ',' expr ',' expr ')'
    |   expr '(' ( expr ( ',' expr )* )?  ')'
    |   expr ('*'|'/') expr
    |   expr ('+'|'-') expr
    |   '[' RATIONAL ( ',' RATIONAL )* ']'
    |   RATIONAL
    |   VARIABLE
    ;
booleanExpr:
         left=expr op=('<'|'<='|'='|'>='|'>') right=expr
    |    op=BOOLEAN
    ;
BOOLEAN : ('true'|'false');
VARIABLE : ([a-z]|[A-Z])([a-z]|[A-Z]|[0-9]|'_')*;
RATIONAL : ('-')?[0-9]*('.')?[0-9]+ ;
WS : ( ' ' | '\t' | '\r' | '\n' )+ {self.skip()};
