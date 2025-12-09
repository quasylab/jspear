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

import it.unicam.quasylab.jspear.ds.DataStateBooleanExpression;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Represents an evolution sequence under the effect of a given perturbation.
 */
public class PerturbedEvolutionSequence extends EvolutionSequence {

    private Perturbation p;

    /**
     * Generates the perturbed version of a given evolution sequence,
     * obtained by applying a given perturbation
     * to a given data state.
     *
     * @param monitor a monitor
     * @param rg a random generator
     * @param sequence an evolution sequence
     * @param perturbedStep initial data state to which the perturbation is applied
     * @param p the perturbation
     * @param scale multiplication factor for the number of samples to be used
     *              in the simulation of the perturbed system.
     */
    protected PerturbedEvolutionSequence(SimulationMonitor monitor, RandomGenerator rg, List<SampleSet<SystemState>> sequence, SampleSet<SystemState> perturbedStep, Perturbation p, int scale) {
        super(monitor, rg, sequence);
        this.p = p;
        doAdd(doApply(perturbedStep.replica(scale)));
    }

    @Override
    protected synchronized SampleSet<SystemState> generateNextStep() {
        this.p = this.p.step();
        return doApply(super.generateNextStep());
    }

    @Override
    public synchronized SampleSet<SystemState> generateNextStepCond(DataStateBooleanExpression condition) {
        this.p = this.p.step();
        return doApply(super.generateNextStepCond(condition));
    }


    /**
     * Applies this perturbation to a given sample set.
     *
     * @param sample a given sample set
     * @return the perturbation of <code>sample</code> via <code>this.p</code>.
     */
    protected synchronized SampleSet<SystemState> doApply(SampleSet<SystemState> sample) {
        Optional<DataStateFunction> perturbationFunction = this.p.effect();
        if (perturbationFunction.isPresent()) {
            return sample.apply(getRandomGenerator(), (rg, s) -> s.apply(rg, perturbationFunction.get()));
        } else {
            return sample;
        }
    }
}
