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

import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;
import it.unicam.quasylab.jspear.robtl.RobustnessFormulaVisitor;
import it.unicam.quasylab.jspear.robtl.RobustnessFunction;

public final class ConjunctionDisTLFormula implements DisTLFormula {

    private final DisTLFormula leftFormula;
    private final DisTLFormula rightFormula;

    public ConjunctionDisTLFormula(DisTLFormula leftFormula, DisTLFormula rightFormula) {
        this.leftFormula = leftFormula;
        this.rightFormula = rightFormula;
    }

    @Override
    public double eval(int sampleSize, int step, EvolutionSequence sequence, boolean parallel) {
        return Math.min(leftFormula.eval(sampleSize, step, sequence, parallel),rightFormula.eval(sampleSize, step, sequence, parallel));
    }

    @Override
    public <Double> DisTLFunction<Double> eval(DisTLFormulaVisitor<Double> evaluator) {
        return evaluator.evalConjunction(this);
    }

    public DisTLFormula getLeftFormula() {
        return leftFormula;
    }

    public DisTLFormula getRightFormula() {
        return rightFormula;
    }

}
