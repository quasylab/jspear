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

import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.penalty.*;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;

import java.util.stream.IntStream;
import java.util.Optional;

public class DoubleSemanticsVisitor implements DisTLFormulaVisitor<Double> {

    private final boolean parallel;

    public DoubleSemanticsVisitor(boolean parallel) {
        this.parallel = parallel;
    }

    public DoubleSemanticsVisitor() {
        this(false);
    }

    @Override
    public DisTLFunction<Double> eval(DisTLFormula formula) {
        return formula.eval(this);
    }

    @Override
    public DisTLFunction<Double> evalAlways(AlwaysDisTLFormula alwaysDisTLFormula) {
        DisTLFunction<Double> argumentFunction = alwaysDisTLFormula.getArgument().eval(this);
        int from = alwaysDisTLFormula.getFrom();
        int to = alwaysDisTLFormula.getTo();
        if (parallel) {
            return (sampleSize, step, sequence) -> IntStream.of(from, to).parallel().mapToDouble(i -> argumentFunction.eval(sampleSize, step+i, sequence)).min().orElse(Double.NaN);
        } else {
            return (sampleSize, step, sequence) -> IntStream.of(from, to).sequential().mapToDouble(i -> argumentFunction.eval(sampleSize, step+i, sequence)).min().orElse(Double.NaN);

        }
    }

    @Override
    public DisTLFunction<Double> evalBrink(BrinkDisTLFormula brinkDisTLFormula){
        DataStateFunction mu = brinkDisTLFormula.getDistribution();
        Optional<DataStateExpression> rho = brinkDisTLFormula.getRho();
        Penalty P = brinkDisTLFormula.getP();
        double q = brinkDisTLFormula.getThreshold();
        if (rho.isPresent()) {
            return (sampleSize, step, sequence)
                    -> sequence.get(step).distanceLeq(rho.get(), sequence.get(step).replica(sampleSize).applyDistribution(new DefaultRandomGenerator(), mu)) - q;
        } else {
            return (sampleSize, step, sequence)
                    -> sequence.get(step).distanceLeq(P, sequence.get(step).replica(sampleSize).applyDistribution(new DefaultRandomGenerator(), mu),step) - q;
        }
    }


    @Override
    public DisTLFunction<Double> evalConjunction(ConjunctionDisTLFormula conjunctionDisTLFormula) {
        DisTLFunction<Double> leftFunction = conjunctionDisTLFormula.getLeftFormula().eval(this);
        DisTLFunction<Double> rightFunction = conjunctionDisTLFormula.getRightFormula().eval(this);
        return (sampleSize, step, sequence) -> Math.min(leftFunction.eval(sampleSize, step, sequence), rightFunction.eval(sampleSize, step, sequence));
    }

    @Override
    public DisTLFunction<Double> evalDisjunction(DisjunctionDisTLFormula disjunctionDisTLFormula) {
        DisTLFunction<Double> leftFunction = disjunctionDisTLFormula.getLeftFormula().eval(this);
        DisTLFunction<Double> rightFunction = disjunctionDisTLFormula.getRightFormula().eval(this);
        return (sampleSize, step, sequence) -> Math.max(leftFunction.eval(sampleSize, step, sequence),rightFunction.eval(sampleSize, step, sequence));
    }

    @Override
    public DisTLFunction<Double> evalEventually(EventuallyDisTLFormula eventuallyDisTLFormula) {
        DisTLFunction<Double> argumentFunction = eventuallyDisTLFormula.getArgument().eval(this);
        int from = eventuallyDisTLFormula.getFrom();
        int to = eventuallyDisTLFormula.getTo();
        if (parallel) {
            return (sampleSize, step, sequence) -> IntStream.of(from, to).parallel().mapToDouble(i -> argumentFunction.eval(sampleSize, step+i, sequence)).max().orElse(Double.NaN);
        } else {
            return (sampleSize, step, sequence) -> IntStream.of(from, to).parallel().mapToDouble(i -> argumentFunction.eval(sampleSize, step+i, sequence)).max().orElse(Double.NaN);
        }
    }

    @Override
    public DisTLFunction<Double> evalFalse() {
        return (sampleSize, step, sequence) -> -1.0;
    }

    @Override
    public DisTLFunction<Double> evalImplication(ImplicationDisTLFormula implicationDisTLFormula) {
        DisTLFunction<Double> leftFunction = implicationDisTLFormula.getLeftFormula().eval(this);
        DisTLFunction<Double> rightFunction = implicationDisTLFormula.getRightFormula().eval(this);
        return (sampleSize, step, sequence) -> Math.max(-leftFunction.eval(sampleSize, step, sequence), rightFunction.eval(sampleSize, step, sequence));
    }

    @Override
    public DisTLFunction<Double> evalNegation(NegationDisTLFormula negationDisTLFormula) {
        DisTLFunction<Double> argumentFunction = negationDisTLFormula.getArgument().eval(this);
        return (sampleSize, step, sequence) -> - argumentFunction.eval(sampleSize, step, sequence);
    }

    @Override
    public DisTLFunction<Double> evalTarget(TargetDisTLFormula targetDisTLFormula) {
        DataStateFunction mu = targetDisTLFormula.getDistribution();
        Optional<DataStateExpression> rho = targetDisTLFormula.getRho();
        Penalty P = targetDisTLFormula.getP();
        double q = targetDisTLFormula.getThreshold();
        if (rho.isPresent()) {
            return (sampleSize, step, sequence)
                    -> q - sequence.get(step).distanceGeq(rho.get(), sequence.get(step).replica(sampleSize).applyDistribution(new DefaultRandomGenerator(), mu));
        } else {
            return (sampleSize, step, sequence)
                    -> q - sequence.get(step).distanceGeq(P, sequence.get(step).replica(sampleSize).applyDistribution(new DefaultRandomGenerator(), mu), step);
        }
    }

    @Override
    public DisTLFunction<Double> evalTrue() {
        return (sampleSize, step, sequence) -> 1.0;
    }

    @Override
    public DisTLFunction<Double> evalUntil(UntilDisTLFormula untilDisTLFormula) {
        DisTLFunction<Double> leftFunction = untilDisTLFormula.getLeftFormula().eval(this);
        DisTLFunction<Double> rightFunction = untilDisTLFormula.getRightFormula().eval(this);
        int from = untilDisTLFormula.getFrom();
        int to = untilDisTLFormula.getTo();
        if (parallel) {
            return (sampleSize, step, sequence) ->
                    IntStream.range(from+step, to+step).sequential().mapToDouble(
                    i -> Math.min(rightFunction.eval(sampleSize, i, sequence),
                            IntStream.range(from+step, i).mapToDouble(j -> leftFunction.eval(sampleSize, j, sequence)).min().orElse(Double.NaN))).max().orElse(Double.NaN);
        } else {
            return (sampleSize, step, sequence) ->
                    IntStream.range(from+step, to+step).sequential().mapToDouble(
                            i -> Math.min(rightFunction.eval(sampleSize, i, sequence),
                                    IntStream.range(from+step, i).mapToDouble(j -> leftFunction.eval(sampleSize, j, sequence)).min().orElse(Double.NaN))).max().orElse(Double.NaN);
        }
    }

}
