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

public final class ConjunctionThreeValuedFormula implements ThreeValuedFormula {

    private final ThreeValuedFormula leftFormula;
    private final ThreeValuedFormula rightFormula;

    public ConjunctionThreeValuedFormula(ThreeValuedFormula leftFormula, ThreeValuedFormula rightFormula) {
        this.leftFormula = leftFormula;
        this.rightFormula = rightFormula;
    }

    @Override
    public TruthValues eval(int sampleSize, int step, EvolutionSequence sequence) {
        return TruthValues.and(leftFormula.eval(sampleSize, step, sequence),rightFormula.eval(sampleSize, step, sequence));
        //if(leftFormula.eval(sampleSize, step, sequence)==TruthValues.FALSE || rightFormula.eval(sampleSize, step, sequence)==TruthValues.FALSE) return TruthValues.FALSE;
        //if(leftFormula.eval(sampleSize, step, sequence)==TruthValues.TRUE && rightFormula.eval(sampleSize, step, sequence)==TruthValues.TRUE) return TruthValues.TRUE;
        //return TruthValues.UNKNOWN;
    }
}
