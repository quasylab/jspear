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

import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * This class implements the three-valued interpretation of RobTL formulae.
 */
public class ThreeValuedSemanticsVisitor implements RobustnessFormulaVisitor<TruthValues> {

    private final RandomGenerator rg;
    private final int m;
    private final double z;

    /**
     * As the evaluation of confidence intervals is necessary to determine the three-valued semantics,
     * the class takes the following two parameters:
     *
     * @param rg random generator
     * @param m number of repetitions for the bootstrap method
     * @param z the quantile of the normal distribution encoding the desired coverage probability.
     */
    public ThreeValuedSemanticsVisitor(RandomGenerator rg, int m, double z) {
        this.rg = rg;
        this.m = m;
        this.z = z;
    }

    /**
     * In case parameters <code>rg</code>, <code>m</code> and <code>z</code> are not declared,
     * default values are used.
     */
    public ThreeValuedSemanticsVisitor() {
        this(new DefaultRandomGenerator(),50,1.96);
    }

    @Override
    public RobustnessFunction<TruthValues> eval(RobustnessFormula formula) {
        return formula.eval(this);
    }

    @Override
    public RobustnessFunction<TruthValues> evalAlways(AlwaysRobustnessFormula alwaysRobustnessFormula) {
        RobustnessFunction<TruthValues> argumentFunction = alwaysRobustnessFormula.getArgument().eval(this);
        int from = alwaysRobustnessFormula.getFrom();
        int to = alwaysRobustnessFormula.getTo();
        return ((sampleSize, step, sequence) -> {
            TruthValues value = TruthValues.TRUE;
            for(int i = from+step; i<to+step; i++){
                value = TruthValues.and(value, argumentFunction.eval(sampleSize, i, sequence));
                if (value == TruthValues.FALSE){
                    i = to+step;
                }
            }
            return value;
        });
    }

    @Override
    public RobustnessFunction<TruthValues> evalAtomic(AtomicRobustnessFormula atomicRobustnessFormula) {
        Perturbation perturbation = atomicRobustnessFormula.getPerturbation();
        DistanceExpression expr = atomicRobustnessFormula.getDistanceExpression();
        RelationOperator relop = atomicRobustnessFormula.getRelationOperator();
        double value = atomicRobustnessFormula.getThreshold();
        return (sampleSize, step, sequence) -> {
            double[] res = expr.evalCI(rg, step, sequence, sequence.apply(perturbation, step, sampleSize), m, z);
            if(res[1] < value && value < res[2]){return TruthValues.UNKNOWN;}
            if(relop.eval(res[0],value)){return TruthValues.TRUE;}
            return TruthValues.FALSE;
        };
    }

    @Override
    public RobustnessFunction<TruthValues> evalConjunction(ConjunctionRobustnessFormula conjunctionRobustnessFormula) {
        RobustnessFunction<TruthValues> leftFunction = conjunctionRobustnessFormula.getLeftFormula().eval(this);
        RobustnessFunction<TruthValues> rightFunction = conjunctionRobustnessFormula.getRightFormula().eval(this);
        return (sampleSize, step, sequence) ->
                TruthValues.and(leftFunction.eval(sampleSize, step, sequence), rightFunction.eval(sampleSize, step, sequence));
    }

    @Override
    public RobustnessFunction<TruthValues> evalDisjunction(DisjunctionRobustnessFormula disjunctionRobustnessFormula) {
        RobustnessFunction<TruthValues> leftFunction = disjunctionRobustnessFormula.getLeftFormula().eval(this);
        RobustnessFunction<TruthValues> rightFunction = disjunctionRobustnessFormula.getRightFormula().eval(this);
        return (sampleSize, step, sequence) ->
                TruthValues.or(leftFunction.eval(sampleSize, step, sequence), rightFunction.eval(sampleSize, step, sequence));
    }

    @Override
    public RobustnessFunction<TruthValues> evalEventually(EventuallyRobustnessFormula eventuallyRobustnessFormula) {
        RobustnessFunction<TruthValues> argumentFunction = eventuallyRobustnessFormula.getArgument().eval(this);
        int from = eventuallyRobustnessFormula.getFrom();
        int to = eventuallyRobustnessFormula.getTo();
        return ((sampleSize, step, sequence) -> {
            TruthValues value = TruthValues.FALSE;
            for(int i = from+step; i<to+step; i++){
                value = TruthValues.or(value, argumentFunction.eval(sampleSize, i, sequence));
                if(value==TruthValues.TRUE){i=to+step;}
            }
            return value;
        });
    }

    @Override
    public RobustnessFunction<TruthValues> evalFalse() {
        return (sampleSize, step, sequence) -> TruthValues.FALSE;
    }

    @Override
    public RobustnessFunction<TruthValues> evalImplication(ImplicationRobustnessFormula implicationRobustnessFormula) {
        RobustnessFunction<TruthValues> leftFunction = implicationRobustnessFormula.getLeftFormula().eval(this);
        RobustnessFunction<TruthValues> rightFunction = implicationRobustnessFormula.getRightFormula().eval(this);
        return (sampleSize, step, sequence) ->
                TruthValues.imply(leftFunction.eval(sampleSize, step, sequence),rightFunction.eval(sampleSize, step, sequence));
    }

    @Override
    public RobustnessFunction<TruthValues> evalNegation(NegationRobustnessFormula negationRobustnessFormula) {
        RobustnessFunction<TruthValues> argumentFunction = negationRobustnessFormula.getArgument().eval(this);
        return (sampleSize, step, sequence) ->
                TruthValues.neg(argumentFunction.eval(sampleSize, step, sequence));
    }

    @Override
    public RobustnessFunction<TruthValues> evalTrue() {
        return (sampleSize, step, sequence) -> TruthValues.TRUE;
    }

    @Override
    public RobustnessFunction<TruthValues> evalUntil(UntilRobustnessFormula untilRobustnessFormula) {
        RobustnessFunction<TruthValues> leftFunction = untilRobustnessFormula.getLeftFormula().eval(this);
        RobustnessFunction<TruthValues> rightFunction = untilRobustnessFormula.getRightFormula().eval(this);
        int from = untilRobustnessFormula.getFrom();
        int to = untilRobustnessFormula.getTo();
        return ((sampleSize, step, sequence) -> {
            TruthValues value = TruthValues.FALSE;
            TruthValues leftValue = TruthValues.TRUE;
            for(int i=from+step; (i<to+step)&&(value!=TruthValues.TRUE)&&(leftValue!=TruthValues.FALSE); i++){
                //double start = System.currentTimeMillis();
                value = TruthValues.and(leftValue, rightFunction.eval(sampleSize, i, sequence));
                if (value != TruthValues.TRUE) {
                    leftValue = TruthValues.and(leftValue, leftFunction.eval(sampleSize, i, sequence));
                }
                //System.out.println(i+"> "+(System.currentTimeMillis()-start));
            }
            return value;
//
//            TruthValues value = TruthValues.FALSE;
//            for(int i=from+step; i<to+step; i++){
//                TruthValues res = TruthValues.TRUE;
//                for(int j=from+step; j<i; j++){
//                    res = TruthValues.and(res, leftFunction.eval(sampleSize, j, sequence));
//                }
//                value = TruthValues.or(value, TruthValues.and(res, rightFunction.eval(sampleSize, i, sequence)));
//            }
//            return value;
        });
    }

}