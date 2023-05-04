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

import java.util.stream.IntStream;

public final class EventuallyDisTLFormula implements DisTLFormula {

    private final DisTLFormula arg;
    private final int from;
    private final int to;


    public EventuallyDisTLFormula(DisTLFormula arg, int from, int to) {
        this.arg = arg;
        this.from = from;
        this.to = to;
    }

    @Override
    public double eval(int sampleSize, int step, EvolutionSequence sequence, boolean parallel) {
        if (parallel) {
            return IntStream.of(from, to).parallel().mapToDouble(i -> arg.eval(sampleSize, step+i, sequence, true)).max().orElse(Double.NaN);
        } else {
            return IntStream.of(from, to).sequential().mapToDouble(i -> arg.eval(sampleSize, step+i, sequence, false)).max().orElse(Double.NaN);

        }
    }

    @Override
    public <Double> DisTLFunction<Double> eval(DisTLFormulaVisitor<Double> evaluator) {
        return evaluator.evalEventually(this);
    }

    public DisTLFormula getArgument() {
        return this.arg;
    }

    public int getFrom() {
        return from;
    }

    public int getTo() {
        return to;
    }
}
