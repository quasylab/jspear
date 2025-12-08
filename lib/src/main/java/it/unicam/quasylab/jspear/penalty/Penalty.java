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

package it.unicam.quasylab.jspear.penalty;

import it.unicam.quasylab.jspear.ds.*;

import java.util.*;

/**
 * This interface model a perturbation applied to a given sample set.
 */
public sealed interface Penalty permits

        AtomicPenalty,
        IterativePenalty,
        NonePenalty,
        SequentialPenalty {


    Penalty NONE = new NonePenalty();

    /**
     * Returns the effect of the penalty at the current step.
     *
     * @return the effect of this penalty at the current step.
     */
    DataStateExpression effect();



    /**
     * Returns the penalty active after one computational step.
     *
     * @return the penalty active after one computational step.
     */
    Penalty next();


    /**
     * Returns true if this penalty has been terminated its effects.
     *
     * @return true if this penalty has been terminated its effects.
     */
    boolean isDone();

    /**
     * Returns the list of effects of this penalty.
     *
     * @return the list of effects of this penalty.
     */
    default List<DataStateExpression> totalEffect(){
        List<DataStateExpression> effects = new ArrayList<>();
        Penalty f = this;
        while (!f.isDone()) {
            effects.add(f.effect());
            f = f.next();
        }
        return effects;
    }

    default List<DataStateExpression> effectUpTo(int step){
        List<DataStateExpression> effects = new ArrayList<>();
        Penalty f = this;
        for (int i=0; i<step+1; i++){
            effects.add(f.effect());
            f = f.next();
        }
        return effects;
    }


    default double apply(DataState state, int step) {
        DataStateExpression effect = totalEffect().get(step);
        return effect.eval(state);
    }


}
