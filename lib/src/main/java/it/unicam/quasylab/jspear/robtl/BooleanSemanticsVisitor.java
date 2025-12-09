/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *              Copyright (C) 2023.
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

package it.unicam.quasylab.jspear.robtl;

import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.perturbation.Perturbation;

import java.util.stream.IntStream;

/**
 * This class implements the Boolean interpretation of RobTL formulae.
 */
public class BooleanSemanticsVisitor implements RobustnessFormulaVisitor<Boolean> {

    private final boolean parallel;

    public BooleanSemanticsVisitor(boolean parallel) {
        this.parallel = parallel;
    }

    public BooleanSemanticsVisitor() {
        this(false);
    }

    @Override
    public RobustnessFunction<Boolean> eval(RobustnessFormula formula) {
        return formula.eval(this);
    }

    @Override
    public RobustnessFunction<Boolean> evalAlways(AlwaysRobustnessFormula alwaysRobustnessFormula) {
        RobustnessFunction<Boolean> argumentFunction = alwaysRobustnessFormula.getArgument().eval(this);
        int from = alwaysRobustnessFormula.getFrom();
        int to = alwaysRobustnessFormula.getTo();
        if (parallel) {
            return (sampleSize, step, sequence) ->
                    IntStream.of(from, to).parallel().allMatch(i -> argumentFunction.eval(sampleSize, step+i, sequence));
        } else {
            return (sampleSize, step, sequence) ->
                    IntStream.of(from, to).sequential().allMatch(i -> argumentFunction.eval(sampleSize, step+i, sequence));
        }
    }

    @Override
    public RobustnessFunction<Boolean> evalAtomic(AtomicRobustnessFormula atomicRobustnessFormula) {
        Perturbation perturbation = atomicRobustnessFormula.getPerturbation();
        DistanceExpression expr = atomicRobustnessFormula.getDistanceExpression();
        RelationOperator relop = atomicRobustnessFormula.getRelationOperator();
        double value = atomicRobustnessFormula.getThreshold();
        return (sampleSize, step, sequence)
                    ->  relop.eval(
                            expr.compute(step, sequence, sequence.apply(perturbation, step, sampleSize)),
                            value
                        );
    }

    @Override
    public RobustnessFunction<Boolean> evalConjunction(ConjunctionRobustnessFormula conjunctionRobustnessFormula) {
        RobustnessFunction<Boolean> leftFunction = conjunctionRobustnessFormula.getLeftFormula().eval(this);
        RobustnessFunction<Boolean> rightFunction = conjunctionRobustnessFormula.getRightFormula().eval(this);
        return (sampleSize, step, sequence) -> leftFunction.eval(sampleSize, step, sequence)&&rightFunction.eval(sampleSize, step, sequence);
    }

    @Override
    public RobustnessFunction<Boolean> evalDisjunction(DisjunctionRobustnessFormula disjunctionRobustnessFormula) {
        RobustnessFunction<Boolean> leftFunction = disjunctionRobustnessFormula.getLeftFormula().eval(this);
        RobustnessFunction<Boolean> rightFunction = disjunctionRobustnessFormula.getRightFormula().eval(this);
        return (sampleSize, step, sequence) -> leftFunction.eval(sampleSize, step, sequence)||rightFunction.eval(sampleSize, step, sequence);
    }

    @Override
    public RobustnessFunction<Boolean> evalEventually(EventuallyRobustnessFormula eventuallyRobustnessFormula) {
        RobustnessFunction<Boolean> argumentFunction = eventuallyRobustnessFormula.getArgument().eval(this);
        int from = eventuallyRobustnessFormula.getFrom();
        int to = eventuallyRobustnessFormula.getTo();
        if (parallel) {
            return (sampleSize, step, sequence) ->
                    IntStream.of(from, to).parallel().anyMatch(i -> argumentFunction.eval(sampleSize, step+i, sequence));
        } else {
            return (sampleSize, step, sequence) ->
                    IntStream.of(from, to).sequential().anyMatch(i -> argumentFunction.eval(sampleSize, step+i, sequence));
        }
    }

    @Override
    public RobustnessFunction<Boolean> evalFalse() {
        return (sampleSize, step, sequence) -> false;
    }

    @Override
    public RobustnessFunction<Boolean> evalImplication(ImplicationRobustnessFormula implicationRobustnessFormula) {
        RobustnessFunction<Boolean> leftFunction = implicationRobustnessFormula.getLeftFormula().eval(this);
        RobustnessFunction<Boolean> rightFunction = implicationRobustnessFormula.getRightFormula().eval(this);
        return (sampleSize, step, sequence) ->
                (!leftFunction.eval(sampleSize, step, sequence))||rightFunction.eval(sampleSize, step, sequence);
    }

    @Override
    public RobustnessFunction<Boolean> evalNegation(NegationRobustnessFormula negationRobustnessFormula) {
        RobustnessFunction<Boolean> argumentFunction = negationRobustnessFormula.getArgument().eval(this);
        return (sampleSize, step, sequence) -> !argumentFunction.eval(sampleSize, step, sequence);
    }

    @Override
    public RobustnessFunction<Boolean> evalTrue() {
        return (sampleSize, step, sequence) -> true;
    }

    @Override
    public RobustnessFunction<Boolean> evalUntil(UntilRobustnessFormula untilRobustnessFormula) {
        RobustnessFunction<Boolean> leftFunction = untilRobustnessFormula.getLeftFormula().eval(this);
        RobustnessFunction<Boolean> rightFunction = untilRobustnessFormula.getRightFormula().eval(this);
        int from = untilRobustnessFormula.getFrom();
        int to = untilRobustnessFormula.getTo();
        if (parallel) {
            return (sampleSize, step, sequence) -> IntStream.range(from+step, to+step).parallel().anyMatch(
                    i -> rightFunction.eval(sampleSize, i, sequence) &&
                            IntStream.range(from+step, i).allMatch(j -> leftFunction.eval(sampleSize, j, sequence))
            );
        } else {
            return (sampleSize, step, sequence) -> IntStream.range(from+step, to+step).sequential().anyMatch(
                    i -> rightFunction.eval(sampleSize, i, sequence) &&
                            IntStream.range(from+step, i).allMatch(j -> leftFunction.eval(sampleSize, j, sequence))
            );
        }
    }

}
