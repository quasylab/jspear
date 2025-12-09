grammar StarkScript;

@header {
    package it.unicam.quasylab.jspear.cli;
}

starkScript : (commands+=scriptCommand '\n')* EOF ;

scriptCommand:
    changeDirectoryCommand
    | listCommand
    | cwdCommand
    | loadCommand
    | quitCommand
    | perturbationsCommand
    | distancesCommand
    | penaltiesCommand
    | formulasCommand
    | evalCommand
    | checkCommand
    | computeCommand
    | saveCommand
    | printCommand
    | clearCommand
    | setCommand
    | infoCommand
;


//helpCommand: 'help';

infoCommand: 'info';

setCommand:
    'set' 'size' '=' value=INTEGER # setSizeCommand
    | 'set' 'm' '=' value=INTEGER # setMCommand
    | 'set' 'z' '=' value=REAL # setZCommand
    | 'set' 'scale' '=' value=INTEGER # setScaleCommand
    | 'set' 'seed' '=' value=INTEGER # setSeedCommand
//    | 'set' 'param' param=ID '=' value=parameterValue # setParameterCommand
    ;

/*
parameterValue:
    INTEGER # integerValue
   | REAL # realValue
   | 'true' # trueValue
   | 'false' # falseValue
   ;
*/

clearCommand: 'clear';

saveCommand: 'save' 'in' target=STRING;

printCommand: 'print';

computeCommand: 'compute' distance=ID 'after' perturbation=ID 'at' when=INTEGER steps=stepExpression;

checkCommand: 'check' semantic=('boolean'|'threevalued') formula=ID steps=stepExpression;

evalCommand: 'eval' penalty=ID steps=stepExpression ;

stepExpression:
    'at' steps+=INTEGER (',' steps+=INTEGER)* # stepExpressionTarget
    | 'from' from=INTEGER 'to' to=INTEGER 'every' step=INTEGER # stepExpressionInterval
;

formulasCommand: 'formulas';

penaltiesCommand: 'penalties';

distancesCommand: 'distances';

perturbationsCommand:
    'perturbations'
;

quitCommand: 'quit';

loadCommand: 'load' target=STRING;

changeDirectoryCommand:
    'cd' target=STRING
;

listCommand:
    'ls'
;

cwdCommand:
    'cwd'
;



fragment DIGIT  :   [0-9];
fragment LETTER :   [a-zA-Z_];
fragment FILE_DIGIT : ~[<>:"/\\|?*\n\r];

ID              :   LETTER (DIGIT|LETTER)*;
NEXT_ID         :   ID '\'';
INTEGER         :   DIGIT+;
REAL            :   ((DIGIT* '.' DIGIT+)|DIGIT+ '.')(('E'|'e')('-')?DIGIT+)?;
STRING          : '"' ( ~["\n\r] | '\\"')* '"' ;

COMMENT
    : '/*' .*? '*/' -> channel(HIDDEN) // match anything between /* and */
    ;

WS  : [ \r\t\u000C\n]+ -> channel(HIDDEN)
    ;