/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
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

import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.robtl.*;

public sealed interface DisTLFormula permits
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
        UntilDisTLFormula {

    default double eval(int sampleSize, int step, EvolutionSequence sequence) {
        return eval(sampleSize, step, sequence, true);
    }

    double eval(int sampleSize, int step, EvolutionSequence sequence, boolean parallel);

    <T> DisTLFunction<T> eval(DisTLFormulaVisitor<T> evaluator);

    static DisTLFunction<Double> getDoubleEvaluationFunction(DisTLFormula formula) {
        return formula.eval(new DoubleSemanticsVisitor());
    }


}