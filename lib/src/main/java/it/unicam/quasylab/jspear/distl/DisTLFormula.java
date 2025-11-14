/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package it.unicam.quasylab.jspear.distl;

import it.unicam.quasylab.jspear.robtl.*;
import it.unicam.quasylab.jspear.udistl.UDisTLFormula;

public sealed interface DisTLFormula extends UDisTLFormula permits
        AlwaysDisTLFormula,
        BrinkDisTLFormula,
        ConjunctionDisTLFormula,
        DisjunctionDisTLFormula,
        EventuallyDisTLFormula,
        FalseDisTLFormula,
        ImplicationDisTLFormula,
        NegationDisTLFormula,
        TargetDisTLFormula,
        TrueDisTLFormula,
        UntilDisTLFormula  {

    <T> DisTLFunction<T> eval(DisTLFormulaVisitor<T> evaluator);

    static DisTLFunction<Double> getDoubleEvaluationFunction(DisTLFormula formula) {
        return formula.eval(new DoubleSemanticsVisitor());
    }

}