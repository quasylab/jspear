grammar JSpearSpecificationLanguage;

@header {
    package it.unicam.quasylab.jspear.speclang;
}


jSpearSpecificationModel : element* EOF ;

element: declarationConstant
| declarationParameter
| declarationVariables
| declarationType
| declarationEnvironmnet
| declarationPenalty
| declarationFunction
| declarationComponent;

/**************************/
/* FUNCTIONS DECLARATIONS */
/**************************/
declarationFunction: 'function' name=ID '(' (arguments+=functionArgument (',' arguments+=functionArgument)*)? ')'
 functionBlockStatement;

functionStatement:
      functionReturnStatement
    | functionIfThenElseStatement
    | functionBlockStatement
    | functionLetStatement
;

functionLetStatement:
    'let' name=ID '=' value=expression 'in' body=functionStatement
;


functionIfThenElseStatement: 'if' '(' guard=expression ')' thenStatement=functionStatement ('else' elseStatement=functionStatement)?;

functionReturnStatement: 'return' expression ';';

functionBlockStatement: '{'  functionStatement '}';

functionArgument: type name=ID;


/******************************/
/* COMPONENTS AND CONTROLLERS */
/******************************/
declarationComponent:
    'component' name=ID '{'
    'variables' '{'
        (variables += variableDeclaration)*
    '}'
    'controller' '{'
        (states += controllerStateDeclaration)*
    '}'
    'init'
        controller = controllerExpression
    '}'
;

controllerStateDeclaration: 'state' name=ID body=controllerBlockBehaviour;

//controllerStateBody:
//    controllerBlockBehaviour
//    | controllerProbabilisticBehaviour
//;

//controllerProbabilisticBehaviour: '{'
//    controllerProbabilisticItem*
//'}';

//controllerProbabilisticItem: ('when' guard=expression)? '[' probability=expression '>' controllerBlockBehaviour ;

controllerBlockBehaviour: '{'
    controllerSequentialBehaviour
'}';

controllerSequentialBehaviour:
    statements+=controllerVariableAssignment* last=controllerTerminalStatement
;


controllerTerminalStatement:
      controllerStepAtion
    | controllerExecAction
    | controllerLetAssignment
//    | controllerSwitchStatement
    | controllerIfThenElseBehaviour
;

//controllerSwitchStatement: 'switch' value = expression '{'
//    cases += controllerCaseStatment*
//    ('default' controllerBlockBehaviour)
//'}';

controllerCaseStatment:
    'case' '(' value = expression ')' controllerBlockBehaviour
;

controllerExpression:
    state = ID                                                      # controllerExpressionReference
    | left= controllerExpression '||' right=controllerExpression    # controllerExpressionParallel
;

/*
systemDeclaration: 'system' name=ID '=' controllerName=ID '{'
    initialAssignments+=initialAssignment*
'}';

initialAssignment: name=ID '=' value=expression ';';
*/

declarationPenalty: 'penalty' name=ID '=' value=expression;






controllerLetAssignment: 'let' name=ID '=' value=expression 'in' body=controllerBlockBehaviour;

controllerVariableAssignment:
('when' guard=expression)? target=varExpression '=' value=expression ';'
;

controllerExecAction: 'exec' target=ID ';';

controllerStepAtion: (steps=expression '#')? 'step' target=ID ';';



controllerIfThenElseBehaviour: 'if' guard=expression thenBranch=controllerBlockBehaviour ('else' elseBranch=controllerBlockBehaviour)?;

/**************************/
/* ENVIRONMENT            */
/**************************/
declarationEnvironmnet:
    'environment' '{'
        ('let' localVariables+=localVariable
        ('and' localVariables+=localVariable)*
        'in')?
        (assignments += variableAssignment)*
    '}'
;


variableAssignment: ('when' guard=expression)? target=varExpression '=' value=expression ';';

varExpression: name=NEXT_ID;// ('[' first=expression (':' last=expression)? ']')?;

localVariable: name=ID '=' expression;

/********************/
/* TYPE DECLARATION */
/********************/
declarationType: 'type' name=ID '=' elements+=typeElementDeclaration ('|' elements += typeElementDeclaration )* ';';

typeElementDeclaration: name=ID;

/********************/
/* GLOBAL VARIABLES */
/********************/
declarationVariables: ('global')? 'variables' '{'
variableDeclaration*
'}';

variableDeclaration: type name=ID ('range' '[' from=expression ',' to=expression ']')? '=' value=expression ';';

type: 'int' #integerType
| 'real' #realType
//| 'array' '[' size=expression ']' #arrayType
| 'bool' #booleanType
| name=ID #customType;

declarationParameter: 'param' name=ID '=' expression ';';

declarationConstant: 'const' name=ID '=' expression ';';

expression:       left=expression op=('&'|'&&') right=expression                      # andExpression
          | left=expression op=('|'|'||') right=expression                      # orExpression
          | left=expression '^' right=expression                                # exponentExpression
          | left=expression op=('*'|'/'|'//') right=expression               # mulDivExpression
          | left=expression op=('+'|'-'|'%') right=expression                   # addSubExpression
          |  left=expression op=('<'|'<='|'=='|'>='|'>') right=expression          # relationExpression
          | '!' arg=expression                                     # negationExpression
          | guard=expression '?' thenBranch=expression ':' elseBranch=expression             # ifThenElseExpression
          | op=('-'|'+') arg=expression                            # unaryExpression
          | '(' expression ')'                                     # bracketExpression
          | INTEGER                                      # intValue
          | REAL                                         # realValue
          | 'false'                                      # falseValue
          | 'true'                                       # trueValue
          | fun=unaryMathFunction '(' argument=expression ')'                   # unaryMathCallExpression
          | fun=binaryMathFunction '(' left=expression ',' right=expression ')' # binaryMathCallExpression
          | name=ID '(' (callArguments += expression (',' callArguments += expression)*)? ')' #callExpression
          | name=ID #referenceExpression //('[' first=expression (':' last=expression)? ']')?
          //| '[' (elements += expression (',' elements += expression)*) ']' #arrayExpression
          | 'N' '[' mean=expression ',' variance=expression ']' #normalExpression
          | 'U' '[' values += expression (',' values += expression)* ']' #uniformExpression
          | 'R' ('[' from = expression ',' to = expression ']')?     #randomExpression/*
          | 'it'                                                     #lambdaParameterExpression
          | target=ID '.' 'count' '(' (guard=expression)? ')' #countArrayElementExpression
          | target=ID '.' 'min' '(' (guard=expression)? ')' #minArrayElementExpression
          | target=ID '.' 'max' '(' (guard=expression)? ')' #maxArrayElementExpression
          | target=ID '.' 'mean' '(' (guard=expression)? ')' #meanArrayElementExpression */
          ;


binaryMathFunction:
    'atan2'
    | 'hypot'
    | 'max'
    | 'min'
    | 'pow'
;

unaryMathFunction: 'abs'
    | 'acos'
    | 'asin'
    | 'atan'
    | 'cbrt'
    | 'ceil'
    | 'cos'
    | 'cosh'
    | 'exp'
    | 'expm1'
    | 'floor'
    | 'log'
    | 'log10'
    | 'log1p'
    | 'signum'
    | 'sin'
    | 'sinh'
    | 'sqrt'
    | 'tan'
    ;



fragment DIGIT  :   [0-9];
fragment LETTER :   [a-zA-Z_];

ID              :   LETTER (DIGIT|LETTER)*;
NEXT_ID         :   ID '\'';
INTEGER         :   DIGIT+;
REAL            :   ((DIGIT* '.' DIGIT+)|DIGIT+ '.')(('E'|'e')('-')?DIGIT+)?;


COMMENT
    : '/*' .*? '*/' -> channel(HIDDEN) // match anything between /* and */
    ;

WS  : [ \r\t\u000C\n]+ -> channel(HIDDEN)
    ;