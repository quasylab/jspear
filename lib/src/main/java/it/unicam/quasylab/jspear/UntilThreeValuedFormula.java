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

import java.util.stream.IntStream;

public final class UntilThreeValuedFormula implements ThreeValuedFormula {

    private final ThreeValuedFormula leftFormula;
    private final int from;
    private final int to;
    private final ThreeValuedFormula rightFormula;

    public UntilThreeValuedFormula(ThreeValuedFormula leftFormula, int from, int to, ThreeValuedFormula rightFormula) {
        if ((from<0)||(to<0)||(from>=to)) {
            throw new IllegalArgumentException();
        }
        this.leftFormula = leftFormula;
        this.from = from;
        this.to = to;
        this.rightFormula = rightFormula;
    }

    @Override
    public TruthValues eval(int sampleSize, int step, EvolutionSequence sequence) {
        TruthValues value = TruthValues.FALSE;
        for(int i=from+step; i<to+step; i++){
            TruthValues res = TruthValues.TRUE;
            for(int j=from+step; j<i; j++){
                res = TruthValues.and(res, leftFormula.eval(sampleSize, j, sequence));
            }
            value = TruthValues.or(value, TruthValues.and(res, rightFormula.eval(sampleSize, i, sequence)));
        }
        return value;
        //return IntStream.range(from+step, to+step).parallel().anyMatch(
        //        i -> ConjunctionThreeValuedFormula(rightFormula.eval(sampleSize, i, sequence),
        //                IntStream.range(step, i).allMatch(j -> leftFormula.eval(sampleSize, j, sequence)))
        //);
    }
}
