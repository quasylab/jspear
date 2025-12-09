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

package it.unicam.quasylab.jspear;

import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;
import it.unicam.quasylab.jspear.robtl.RobustnessFunction;
import it.unicam.quasylab.jspear.robtl.TruthValues;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.Map;
import java.util.stream.IntStream;

/**
 * The class provides the methods necessary to
 * carry out the analysis on a given system specification.
 */
public class SystemSpecification {

    public final static int DEFAULT_SIZE = 1;
    public final static int DEFAULT_M = 50;
    public final static double DEFAULT_Z = 1.96;
    private final ControlledSystem system;
    private final Map<String, DataStateExpression> penalties;
    private final Map<String, RobustnessFormula> formulas;
    private EvolutionSequence sequence;
    private int size = DEFAULT_SIZE;
    private final Map<String, Perturbation> perturbations;
    private final Map<String, DistanceExpression> expressions;
    private int m = 50;
    private double z = 1.96;
    private RandomGenerator rand = new DefaultRandomGenerator();
    private int scale = 10;

    /**
     * Generates a system specification from the following parameters:
     *
     * @param system a system in the evolution sequence model
     * @param penalties a set of penalty function IDs
     * @param formulas a set of RobTL formulae IDs
     * @param perturbations a set of perturbation IDs
     * @param expressions a set of distance expression IDs
     */
    public SystemSpecification(ControlledSystem system, Map<String, DataStateExpression> penalties, Map<String, RobustnessFormula> formulas, Map<String, Perturbation> perturbations, Map<String, DistanceExpression> expressions) {
        this.system = system;
        this.penalties = penalties;
        this.formulas = formulas;
        this.perturbations = perturbations;
        this.expressions = expressions;
    }

    /**
     * Returns the set of penalty function IDs
     *
     * @return the <code>String[]</code> of the names of the penalty functions in the specification.
     */
    public String[] getPenalties() {
        return penalties.keySet().toArray(new String[0]);
    }

    /**
     * Returns the penalty function with the given ID.
     *
     * @param name ID of the penalty
     * @return the penalty function in the specification corresponding to <code>name</code>.
     */
    public DataStateExpression getPenalty(String name) {
        return penalties.get(name);
    }

    /**
     * Returns the system in the evolution sequence model.
     *
     * @return parameter <code>system</code>.
     */
    public ControlledSystem getSystem() {
        return system;
    }

    /**
     * Returns the set of RobTL formulae IDs
     *
     * @return the <code>String[]</code> of the names of the RobTL formulae in the specification.
     */
    public String[] getFormulas() { return formulas.keySet().toArray(new String[0]); }

    /**
     * Returns the RobTL formula with the given ID.
     *
     * @param name ID of the RobTL formula
     * @return the RobTL formula in the specification corresponding to <code>name</code>.
     */
    public RobustnessFormula getFormula(String name) {
        return formulas.get(name);
    }

    /**
     * Returns the set of perturbation IDs
     *
     * @return the <code>String[]</code> of the names of the perturbations in the specification.
     */
    public String[] getPerturbations() {
        return perturbations.keySet().toArray(new String[0]);
    }

    /**
     * Returns the perturbation with the given ID.
     *
     * @param name ID of the perturbation
     * @return the perturbation in the specification corresponding to <code>name</code>.
     */
    public Perturbation getPerturbation(String name) {
        return perturbations.get(name);
    }

    /**
     * Returns the set of distance expression IDs
     *
     * @return the <code>String[]</code> of the names of the distance expressions in the specification.
     */
    public String[] getDistanceExpressions() {
        return expressions.keySet().toArray(new String[0]);
    }

    /**
     * Returns the distance expression with the given ID.
     *
     * @param name ID of the distance expression
     * @return the distance expression in the specification corresponding to <code>name</code>.
     */
    public DistanceExpression getDistanceExpression(String name) {
        return expressions.get(name);
    }

    /**
     * Generates an evolution sequence using as parameters
     * this random generator,
     * this system, and
     * this size.
     */
    public void generateSequence() {
        this.sequence = new EvolutionSequence(rand, rg -> system, this.size);
    }

    /**
     * Sets the size of the sample sets.
     * @param size size of the sample sets.
     */
    public void setSize(int size) {
        this.size = size;
    }

    /**
     * Sets the basic parameters of a given evaluator for RobTL formulae on a given interpretation domain.
     *
     * @param evaluationFunction method for the evaluation of RobTL formulae
     * @param sampleSize size of the sample set used for the simulation
     * @param step time step at which the evaluation starts
     * @return the evaluator set with the given parameters
     * @param <T> interpretation domain
     */
    private <T> T eval(RobustnessFunction<T> evaluationFunction, int sampleSize, int step) {
        return evaluationFunction.eval(sampleSize, step, getSequence());
    }

    /**
     * Stores the results of the evaluator,
     * computed every <code>by</code> steps starting from <code>from</code>,
     * in an array on a given interpretation domain.
     *
     * @param evaluationFunction an evaluator for RobTL formulae
     * @param sampleSize size of the sample set used in the simulation
     * @param from starting time steps for the evaluation
     * @param by evaluation step
     * @param data an array of values in a given interpretation domain
     * @param <T> interpretation domain
     */
    private <T> void eval(RobustnessFunction<T> evaluationFunction, int sampleSize, int from, int by, T[] data) {
        for(int i=0; i<data.length; i++) {
            data[i] = eval(evaluationFunction, sampleSize, from+by*i);
        }
    }

    /**
     * Stores the results of the evaluator,
     * computed at given time steps,
     * in an array on a given interpretation domain.
     *
     * @param evaluationFunction an evaluator for RobTL formulae
     * @param sampleSize size of the sample set used in the simulation
     * @param steps time steps at which the evaluation is done
     * @param data an array of values in a given interpretation domain
     * @param <T> interpretation domain
     */
    private <T> void eval(RobustnessFunction<T> evaluationFunction, int sampleSize, int[] steps, T[] data) {
        for(int i=0; i<data.length; i++) {
            data[i] = eval(evaluationFunction, sampleSize, steps[i]);
        }
    }

    /**
     * Returns the Boolean evaluation of the RobTL formula with the given ID at a given time step.
     *
     * @param name ID of the RobTL formula
     * @param sampleSize size of the sample set in the simulation
     * @param step time step at which the formula is evaluated
     * @return the Boolean evaluation of the RobTL formula corresponding to <code>name</code> at time <code>step</code>.
     */
    public boolean evalBooleanSemantic(String name, int sampleSize, int step) {
        RobustnessFormula formula = getFormula(name);
        if (formula == null) {
            return false;
        }
        return eval(RobustnessFormula.getBooleanEvaluationFunction(formula), sampleSize, step);
    }

    /**
     * Returns the evaluations, in Boolean semantics, of the RobTl formula with the given ID,
     * at a given sequence of time steps.
     *
     * @param name ID of the RobTL formula
     * @param sampleSize size of the sample set in the simulation
     * @param from starting evaluation time step
     * @param to final evaluation time step
     * @param by evaluation step
     * @return the array containing the Boolean evaluations of the RobTL formula corresponding to <code>name</code>
     * computed every <code>by</code> steps in the time interval <code>[from,to]</code>.
     */
    public Boolean[] evalBooleanSemantic(String name, int sampleSize, int from, int to, int by) {
        RobustnessFormula formula = getFormula(name);
        if (formula == null) {
            return null;
        }
        Boolean[] data = new Boolean[(to-from)/by];
        eval(RobustnessFormula.getBooleanEvaluationFunction(formula), sampleSize, from, by, data);
        return data;
    }

    /**
     * Returns the evaluations, in Boolean semantics, of the RobTl formula with the given ID,
     * at given time steps.
     *
     * @param name ID of the RobTL formula
     * @param sampleSize size of the sample set in the simulation
     * @param steps time steps at which the formula is evaluated
     * @return the array containing the Boolean evaluations of the RobTL formula corresponding to <code>name</code>
     * computed at the time steps indicated in <code>steps</code>.
     */
    public Boolean[] evalBooleanSemantic(String name, int sampleSize, int[] steps) {
        RobustnessFormula formula = getFormula(name);
        if (formula == null) {
            return null;
        }
        Boolean[] data = new Boolean[steps.length];
        eval(RobustnessFormula.getBooleanEvaluationFunction(formula), sampleSize, steps, data);
        return data;
    }

    /**
     * Returns the three-valued evaluation of the RobTL formula with the given ID at a given time step.
     *
     * @param name ID of the RobTL formula
     * @param sampleSize size of the sample set in the simulation
     * @param step time step at which the formula is evaluated
     * @return the three-valued evaluation of the RobTL formula corresponding to <code>name</code> at time <code>step</code>.
     */
    public TruthValues evalThreeValuedSemantic(String name, int sampleSize, int step) {
        RobustnessFormula formula = getFormula(name);
        if (formula == null) {
            return TruthValues.FALSE;
        }
        return eval(RobustnessFormula.getThreeValuedEvaluationFunction(rand, m, z, formula), sampleSize, step);
    }

    /**
     * Returns the evaluations, in three-valued semantics, of the RobTl formula with the given ID,
     * at a given sequence of time steps.
     *
     * @param name ID of the RobTL formula
     * @param sampleSize size of the sample set in the simulation
     * @param from starting evaluation time step
     * @param to final evaluation time step
     * @param by evaluation step
     * @return the array containing the three-valued evaluations of the RobTL formula corresponding to <code>name</code>
     * computed every <code>by</code> steps in the time interval <code>[from,to]</code>.
     */
    public TruthValues[] evalThreeValuedSemantic(String name, int sampleSize, int from, int to, int by) {
        RobustnessFormula formula = getFormula(name);
        if (formula == null) {
            return null;
        }
        TruthValues[] data = new TruthValues[(to-from)/by];
        eval(RobustnessFormula.getThreeValuedEvaluationFunction(rand, m, z, formula), sampleSize, from, by, data);
        return data;
    }

    /**
     * Returns the evaluations, in three-valued semantics, of the RobTl formula with the given ID,
     * at given time steps.
     *
     * @param name ID of the RobTL formula
     * @param sampleSize size of the sample set in the simulation
     * @param steps time steps at which the formula is evaluated
     * @return the array containing the three-valued evaluations of the RobTL formula corresponding to <code>name</code>
     * computed at the time steps indicated in <code>steps</code>.
     */
    public TruthValues[] evalThreeValuedSemantic(String name, int sampleSize, int[] steps) {
        RobustnessFormula formula = getFormula(name);
        if (formula == null) {
            return null;
        }
        TruthValues[] data = new TruthValues[steps.length];
        eval(RobustnessFormula.getThreeValuedEvaluationFunction(rand, m, z, formula), sampleSize, steps, data);
        return data;
    }

    /**
     * Returns, or generate, an evolution sequence
     * using the parameters given in the specification.
     *
     * @return this sequence, if already generated.
     * A newly generated sequence, otherwise.
     */
    public EvolutionSequence getSequence() {
        if (this.sequence == null) {
            generateSequence();
        }
        return this.sequence;
    }

    /**
     * Returns the number of samples used for the simulations.
     *
     * @return parameter <code>size</code>.
     */
    public int getSize() {
        return size;
    }

    /**
     * Returns the number of repetitions for the bootstrap method.
     *
     * @return parameter <code>m</code>.
     */
    public int getM() {
        return m;
    }

    /**
     * Returns the quantile of the standard-normal distribution for the boostrap method.
     *
     * @return parameter <code>z</code>.
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets a new value for this number of repetitions for the bootstrap method.
     *
     * @param m new value for <code>m</code>.
     */
    public void setM(int m) {
        this.m = m;
    }

    /**
     * Sets a new value for this quantile of the standard-normal distribution for the bootstrap method.
     *
     * @param z new value for <code>z</code>.
     */
    public void setZ(double z) {
        this.z = z;
    }

    public void setRand(long seed){
        this.rand.setSeed(seed);
    }

    /**
     * Returns the sample of the distribution reached by this sequence at a given time step.
     *
     * @param step time step
     * @return the sample of the distribution reached by this sequence at the time step <code>step</code>.
     */
    public SampleSet<SystemState> getSamplesAt(int step) {
        return getSequence().get(step);
    }

    /**
     * Generates the perturbed version of this sequence obtain by applying the perturbation with the given ID
     * from a given time step, using a given number of samples.
     * The perturbed is generated till a given time step.
     *
     * @param name ID of the perturbation
     * @param step time step at which the perturbation is applied
     * @param scale number of samples for the simulation of the perturbation
     * @param deadline time step of the last generated step in the perturbed sequence
     * @return the perturbation of this sequence,
     * obtained by applying the perturbation corresponding to <code>name</code>
     * at time <code>step</code>,
     * and generated till time <code>deadline</code>.
     */
    public EvolutionSequence applyPerturbation(String name, int step, int scale, int deadline) {
        EvolutionSequence perturbed = getSequence().apply(getPerturbation(name), step, scale);
        perturbed.generateUpTo(deadline);
        return perturbed;
    }

    /**
     * Returns the evaluation of a distance expression with a given ID
     * between this sequence and its perturbation, obtained by applying
     * the perturbation with a given ID,
     * at a given time step.
     *
     * @param expressionName ID of the distance expression
     * @param perturbationName ID of the perturbation
     * @param step time step at which the perturbation is applied, and at which the evaluation of the distance expression starts
     * @param scale number of samples for the simulation of the perturbed sequence
     * @return the evaluation, at time <code>step</code>,
     * of the distance expression corresponding to <code>expressionName</code>
     * between this sequence and its perturbation obtained by applying
     * the perturbation corresponding to <code>perturbationName</code>
     * at time <code>step</code>.
     */
    public double evalDistanceExpression(String expressionName, String perturbationName, int step, int scale) {
        EvolutionSequence perturbed = getSequence().apply(getPerturbation(perturbationName), step, scale);
        DistanceExpression expr = getDistanceExpression(expressionName);
        return expr.compute(step, getSequence(), perturbed);
    }

    /**
     * Returns the evaluations of a distance expression with a given ID
     * between this sequence and its perturbation, obtained by applying
     * the perturbation with a given ID at given time step,
     * at given time steps.
     *
     * @param expressionName ID of the distance expression
     * @param perturbationName ID of the perturbation
     * @param perturbationStep time step at which the perturbation is applied
     * @param scale number of samples for the simulation of the perturbed sequence
     * @param steps time steps at which the distance expression is evaluated
     * @return the evaluations, at time steps in <code>steps</code>,
     * of the distance expression corresponding to <code>expressionName</code>
     * between this sequence and its perturbation obtained by applying
     * the perturbation corresponding to <code>perturbationName</code>
     * at time <code>perturbationStep</code>.
     */
    public double[] evalDistanceExpression(String expressionName, String perturbationName, int perturbationStep, int scale, int[] steps) {
        EvolutionSequence perturbed = getSequence().apply(getPerturbation(perturbationName), perturbationStep, scale);
        DistanceExpression expr = getDistanceExpression(expressionName);
        return IntStream.of(steps).mapToDouble(i -> expr.compute(i, getSequence(), perturbed)).toArray();
    }

    /**
     * Returns the evaluation of the penalty function with a given ID
     * at a given time step.
     *
     * @param name ID of the penalty function
     * @param step time step at which the penalty function is evaluated
     * @return the evaluation of the penalty function corresponding to <code>name</code>
     * on this sequence
     * at time step <code>step</code>.
     */
    public double[] evalPenalty(String name, int step) {
        DataStateExpression f = penalties.get(name);
        if (f == null) {
            return new double[0];
        } else {
            return getSequence().get(step).evalPenaltyFunction(f);
        }
    }

    /**
     * Returns the evaluations of the penalty function with a given ID
     * at given time steps.
     *
     * @param name ID of the penalty function
     * @param steps time steps at which the penalty function is evaluated
     * @return the evaluations of the penalty function corresponding to <code>name</code>
     * on this sequence
     * at the time steps in <code>steps</code>.
     */
    public double[][] evalPenalty(String name, int[] steps) {
        DataStateExpression f = penalties.get(name);
        if (f == null) {
            return new double[0][0];
        } else {
            return IntStream.of(steps).sequential().mapToObj(i -> getSequence().get(i).evalPenaltyFunction(f)).toArray(double[][]::new);
        }
    }

    /**
     * Resets the default parameters.
     */
    public void clear() {
        this.sequence = null;
        this.size = DEFAULT_SIZE;
        this.m = DEFAULT_M;
        this.z = DEFAULT_Z;
    }

}
