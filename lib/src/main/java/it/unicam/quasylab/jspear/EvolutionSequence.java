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

import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.ds.DataStateBooleanExpression;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * This class represent a collection of sequences of data sampled from a model described in terms
 * of a {@link DataStateFunction}.
 */
public class EvolutionSequence {

    protected       SampleSet<SystemState>              lastGenerated;
    private final   ArrayList<SampleSet<SystemState>>   sequence;
    private final   RandomGenerator                     rg;
    private final   SimulationMonitor                   monitor;

    /**
     * Creates an evolution sequence originating from the given generator.
     *
     * @param monitor monitor used to control generation of evolution sequence;
     * @param rg random generator;
     * @param generator function used to generate the initial states of the evolution sequence;
     * @param size number of samplings at each time step.
     */
    public EvolutionSequence(SimulationMonitor monitor, RandomGenerator rg, Function<RandomGenerator, SystemState> generator, int size) {
        this.lastGenerated = SampleSet.generate(rg, generator, size);
        this.sequence = new ArrayList<>();
        this.rg = rg;
        this.monitor = monitor;
        this.sequence.add(lastGenerated);
    }


    /**
     * Creates an evolution sequence originating from the given generator.
     *
     * @param rg random generator;
     * @param generator function used to generate the initial states of the evolution sequence;
     * @param size number of samplings at each time step.
     */
    public EvolutionSequence(RandomGenerator rg, Function<RandomGenerator, SystemState> generator, int size) {
        this(null, rg, generator, size);
    }


    /**
     * Creates an evolution sequence whose first elements are contained in the given sequence.
     * @throws IllegalArgumentException if sequence is empty.
     */
    protected EvolutionSequence(SimulationMonitor monitor, RandomGenerator rg, List<SampleSet<SystemState>> sequence) {
        this.sequence = new ArrayList<>(sequence);
        if (!sequence.isEmpty()) {
            this.lastGenerated = this.sequence.get(this.sequence.size()-1);
        }
        this.rg = rg;
        this.monitor = monitor;
    }

    /**
     * Creates an evolution sequence that shares the first <code>steps</code> with the given one.
     *
     * @param originalSequence an evolution sequence.
     * @param steps number of steps to copy.
     * @throws IllegalArgumentException if <code>steps<0</code>.
     */
    protected EvolutionSequence(EvolutionSequence originalSequence, int steps) {
        this(originalSequence.monitor, originalSequence.rg, originalSequence.select(steps));
    }

    /**
     * Returns the list of sample sets of this sequence in the given range (extremes included). If <code>to</code> is negative,
     *
     * @param from first selected step
     * @param to last selected step
     * @return the list of sample sets of this sequence in the given range (extremes included)
     */
    public List<SampleSet<SystemState>> select(int from, int to) {
        if (to<0) {
            return List.of();
        }
        generateUpTo(to);
        return this.sequence.stream().skip(Math.max(0,from)).limit(Math.max(0,1+to-from)).toList();
    }

    /**
     * Returns the list of sample sets of this sequence containing the first <code>n</code> steps.
     *
     * @param n number of selected steps.
     * @return the list of sample sets of this sequence containing the first <code>n</code> steps.
     */
    protected List<SampleSet<SystemState>> select(int n) {
        return this.select(0, n);
    }

    /**
     * Returns the length of the evolution sequence.
     *
     * @return the length of the evolution sequence.
     */
    public int length() {
        return sequence.size();
    }

    /**
     * Returns the sample set at the given step.
     *
     * @param i step index.
     * @return the sample set at the given step.
     * @throws IndexOutOfBoundsException if <code>((i<0)||(i>=length()))</code>.
     */
    public SampleSet<SystemState> get(int i) {
        if (getLastGeneratedStep()<i) {
            generateUpTo(i);
        }
        return sequence.get(i);
    }

    /**
     * Returns the index of last generated step.
     *
     * @return the index of last generated step.
     */
    private int getLastGeneratedStep() {
        return sequence.size()-1;
    }

    /**
     * This method is used to generate the evolution sequence up to the given index.
     *
     * @param n index of the last generated samplings.
     */
    public synchronized void generateUpTo(int n) {
        while (getLastGeneratedStep()<n) {
            int lastGeneratedStep = getLastGeneratedStep();
            startSamplingsOfStep(lastGeneratedStep);
            doAdd( generateNextStep() );
            endSamplingsOfStep(lastGeneratedStep);
        }
    }

    /**
     * This method is used to generate the evolution sequence up to certain conditions.
     *
     * @param conditions list of conditions to be checked.
     */
    public synchronized void generateUpToCond(ArrayList<DataStateBooleanExpression> conditions) {
        while (!conditions.isEmpty()) {
            int lastGeneratedStep = getLastGeneratedStep();
            startSamplingsOfStep(lastGeneratedStep);
            doAdd(generateNextStepCond(conditions.get(0)));
            conditions.remove(0);
            endSamplingsOfStep(lastGeneratedStep);
        }
    }

    protected void doAdd(SampleSet<SystemState> sampling) {
        lastGenerated = sampling;
        sequence.add(lastGenerated);
    }

    protected SampleSet<SystemState> generateNextStep() {
        return lastGenerated.apply(s -> s.sampleNext(rg));
    }

    public SampleSet<SystemState> generateNextStepCond(DataStateBooleanExpression condition) {
        return lastGenerated.apply(s -> s.sampleNextCond(rg,condition));
    }

    /**
     * Utility method used to notify the monitor that the generation of a step is started.
     *
     * @param step sequence step.
     */
    protected void endSamplingsOfStep(int step) {
        if (monitor != null) {
            monitor.endSamplingsOfStep(step);
        }
    }

    /**
     * Utility method used to notify the monitor that the generation of a step is completed.
     *
     * @param step sequence step.
     */
    protected void startSamplingsOfStep(int step) {
        if (monitor != null) {
            monitor.startSamplingsOfStep(step);
        }
    }


    /**
     * Returns the evaluation of the given penalty function at the given time step.
     *
     * @return the evaluation of the given penalty function at the given time step.
     */
    public double[] evalPenaltyFunction(DataStateExpression f, int t) {
        return get(t).evalPenaltyFunction(f);
    }

    /**
     * Returns the random generator used to sample steps of this evolution sequence.
     *
     * @return the random generator used to sample steps of this evolution sequence.
     */
    protected RandomGenerator getRandomGenerator() {
        return rg;
    }


    /**
     * Returns the evolution sequence obtained from this evolution sequence by applying the given
     * perturbation at the given step and by considering the given scale of samplings.
     *
     * @param perturbation perturbation applied to this sequence.
     * @param perturbedStep perturbed step.
     * @param scale scale factor of perturbed sequence.
     * @return the evolution sequence obtained from this evolution sequence by applying the given
     * perturbation at the given step and by considering the given scale of samplings.
     */
    public EvolutionSequence apply(Perturbation perturbation, int perturbedStep, int scale) {
        if (perturbedStep<0) {
            throw new IllegalArgumentException();
        }
        return new PerturbedEvolutionSequence(this.monitor, this.rg, this.select(perturbedStep-1), this.get(perturbedStep), perturbation, scale);
    }




    public double[] compute(Perturbation perturbation, int from, int size, DistanceExpression expr, int t1, int t2) {
        return expr.compute(t1, t2, this, this.apply(perturbation, from, size));
    }



}
