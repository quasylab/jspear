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

import java.util.stream.IntStream;

public final class UntilDisTLFormula implements DisTLFormula {

    private final DisTLFormula leftFormula;
    private final int from;
    private final int to;
    private final DisTLFormula rightFormula;

    public UntilDisTLFormula(DisTLFormula leftFormula, int from, int to, DisTLFormula rightFormula) {
        if ((from<0)||(to<0)||(from>=to)) {
            throw new IllegalArgumentException();
        }
        this.leftFormula = leftFormula;
        this.from = from;
        this.to = to;
        this.rightFormula = rightFormula;
    }

    @Override
    public double eval(int sampleSize, int step, EvolutionSequence sequence, boolean parallel) {
        if (parallel) {
            return IntStream.range(from+step, to+step).sequential().mapToDouble(
                    i -> Math.min(rightFormula.eval(sampleSize, i, sequence, true),
                            IntStream.range(from+step, i).mapToDouble(j -> leftFormula.eval(sampleSize, j, sequence, true)).min().orElse(Double.NaN))).max().orElse(Double.NaN);
        } else {
            return IntStream.range(from+step, to+step).sequential().mapToDouble(
                    i -> Math.min(rightFormula.eval(sampleSize, i, sequence, false),
                            IntStream.range(from+step, i).mapToDouble(j -> leftFormula.eval(sampleSize, j, sequence, false)).min().orElse(Double.NaN))).max().orElse(Double.NaN);
        }
    }

    @Override
    public <Double> DisTLFunction<Double> eval(DisTLFormulaVisitor<Double> evaluator) {
        return evaluator.evalUntil(this);
    }

    public DisTLFormula getLeftFormula() {
        return leftFormula;
    }

    public int getFrom() {
        return from;
    }

    public int getTo() {
        return to;
    }

    public DisTLFormula getRightFormula() {
        return rightFormula;
    }
}
