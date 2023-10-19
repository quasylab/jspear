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

package it.unicam.quasylab.jspear.controller;

import it.unicam.quasylab.jspear.ds.DataStateUpdate;

import java.util.LinkedList;
import java.util.List;
import java.util.function.BinaryOperator;
import java.util.function.UnaryOperator;
import java.util.stream.Stream;

/**
 * Identifies a step executed by a controller.
 */
public record EffectStep<T>(List<DataStateUpdate> effect, T next) {

    /**
     * Applies a given operator on controllers to this step.
     *
     * @param op operator to apply.
     * @return a new step where the next controller is obtained by this by applying the giving operator.
     */
    public EffectStep<T> apply(UnaryOperator<T> op) {
        return new EffectStep<>(effect, op.apply(next));
    }

    /**
     * Returns a step consisting of the parallel application of this step with the one given as parameters.
     *
     * @param other another controller step.
     * @return a step consisting of the parallel application of this step with the one given as parameters.
     */
    public EffectStep<T> parallel(BinaryOperator<T> stepOperator, EffectStep<T> other) {
        List<DataStateUpdate> newEffects = new LinkedList<>();
        newEffects.addAll(this.effect);
        newEffects.addAll(other.effect);
        return new EffectStep<>(newEffects, stepOperator.apply(this.next, other.next));
    }

    /**
     * Returns the concatenation of a customary list of updates with this step.
     *
     * @param updates the list of updates to be applied before this step.
     * @return a step consisting of the concatenation of the given updates with the application of this step.
     */
    public EffectStep<T> applyBefore(List<DataStateUpdate> updates) {
        if (updates.isEmpty()) {
            return this;
        } else {
            return new EffectStep<>(Stream.concat(updates.stream(), this.effect.stream()).toList(), next);
        }
    }
}
