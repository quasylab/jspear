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

package it.unicam.quasylab.jspear;

import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.RobustnessFormula;

import java.util.stream.IntStream;

public final class AlwaysThreeValuedFormula implements ThreeValuedFormula {

    private final ThreeValuedFormula formula;
    private final int from;
    private final int to;

    public AlwaysThreeValuedFormula(ThreeValuedFormula formula, int from, int to) {
        this.formula = formula;
        this.from = from;
        this.to = to;
    }

    @Override
    public TruthValues eval(int sampleSize, int step, EvolutionSequence sequence) {
        TruthValues value = TruthValues.TRUE;
        for(int i = from+step; i<to+step; i++){
            value = TruthValues.and(value, formula.eval(sampleSize, i, sequence));
        }
        return value;
        //return IntStream.of(from, to).parallel().allMatch(i -> formula.eval(sampleSize, step+i, sequence));
    }
}
