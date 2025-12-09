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

import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateBooleanExpression;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import org.apache.commons.math3.random.RandomGenerator;

/**
 * Represent a system in the evolution sequence model
 * in which the evolution is affected by a given perturbation.
 */
public class PerturbedSystem implements SystemState {

    private final SystemState perturbedSystem;
    private final Perturbation perturbation;

    /**
     * Generates a perturbed system, starting from a given system,
     * given a perturbation.
     *
     * @param perturbedSystem a system
     * @param perturbation a perturbation.
     */
    public PerturbedSystem(SystemState perturbedSystem, Perturbation perturbation) {
        this.perturbedSystem = perturbedSystem;
        this.perturbation = perturbation;
    }

    @Override
    public DataState getDataState() {
        return perturbedSystem.getDataState();
    }

    @Override
    public SystemState sampleNext(RandomGenerator rg) {
        SystemState next = perturbedSystem.sampleNext(rg);
        return perturbation.step().apply(rg, next);
    }

    @Override
    public SystemState sampleNextCond(RandomGenerator rg, DataStateBooleanExpression condition) {
        SystemState next = perturbedSystem.sampleNextCond(rg,condition);
        return perturbation.step().apply(rg, next);
    }

    @Override
    public SystemState setDataState(DataState dataState) {
        return new PerturbedSystem(perturbedSystem.setDataState(dataState), perturbation);
    }
}
