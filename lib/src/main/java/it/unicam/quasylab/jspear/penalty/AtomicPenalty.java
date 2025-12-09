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
 * An atomic penalty is used to apply a given penalty function.
 *
 * @param penaltyFunction penalty function to apply.
 */
public record AtomicPenalty(int afterSteps, DataStateExpression penaltyFunction) implements Penalty {

    @Override
    public DataStateExpression effect() {
        if (afterSteps <= 0) {
            return penaltyFunction;
        } else {
            return ds->0.0;
        }
    }

    @Override
    public Penalty next() {
        if (afterSteps <= 0) {
            return NONE;
        } else {
            return new AtomicPenalty(afterSteps-1, penaltyFunction);
        }
    }

    @Override
    public boolean isDone() {
        return false;
    }


}
