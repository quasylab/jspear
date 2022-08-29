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

import it.unicam.quasylab.jspear.ds.DataStateFunction;

import java.util.Optional;

//TODO: Add comments.
public final class AfterPerturbation implements Perturbation {
    private final int steps;
    private final IterativePerturbation body;

    public AfterPerturbation(int steps, IterativePerturbation body) {
        this.steps = steps;
        this.body = body;
    }

    @Override
    public Optional<DataStateFunction> effect() {
        return Optional.empty();
    }

    @Override
    public Perturbation step() {
        if (steps > 1) {
            return new AfterPerturbation(steps-1, body);
        } else {
            return body;
        }
    }

    @Override
    public boolean isDone() {
        return false;
    }
}
