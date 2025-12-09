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

package it.unicam.quasylab.jspear.speclang.controller;

import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.EffectStep;
import it.unicam.quasylab.jspear.speclang.semantics.JSpearExpressionEvaluationFunction;
import it.unicam.quasylab.jspear.speclang.values.JSpearValue;
import it.unicam.quasylab.jspear.speclang.variables.JSpearStore;
import it.unicam.quasylab.jspear.speclang.variables.JSpearVariableAllocation;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.List;

@FunctionalInterface
public interface JSpearControllerFunction {


    EffectStep<Controller> apply(RandomGenerator rg, JSpearStore store);


    static Controller toController(JSpearVariableAllocation allocation, JSpearControllerFunction function) {
        return (rg, ds) -> function.apply(rg, JSpearStore.storeOf(allocation, ds));
    }

    static JSpearControllerFunction ifThenElse(JSpearExpressionEvaluationFunction guard, JSpearControllerFunction thenCase, JSpearControllerFunction elseCase) {
        return (rg, s) -> {
            if (JSpearValue.isTrue(guard.eval(rg, s))) {
                return thenCase.apply(rg, s);
            } else {
                return elseCase.apply(rg, s);
            }
        };
    }

    static JSpearControllerFunction sequential(List<JSpearControllerFunction> functions) {
        return (rg, s) -> {
            EffectStep<Controller> effect = new EffectStep<>(List.of(), null);
            for (JSpearControllerFunction function : functions) {
                effect = effect.applyAfter(function.apply(rg, s));
                if (effect.isCompleted()) return effect;
            }
            return effect;
        };
    }

}
