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

package it.unicam.quasylab.jspear.feedback;

import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.SampleSet;
import it.unicam.quasylab.jspear.SystemState;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import java.nio.file.attribute.DosFileAttributeView;
import java.util.List;
import java.util.function.BiFunction;

/**
 * Instances of this interface are used to represent a random function from data states to data states.
 */
@FunctionalInterface
public interface FeedbackFunction {

    List<DataStateUpdate> apply(RandomGenerator rg, DataState ds, EvolutionSequence sequence);

    /*
    static DataStateFunction apply(int[][] varW, EvolutionSequence sequence) {
        return (rg,ds) -> {
          int step = ds.getStep();
            SampleSet<SystemState> systemStateSampleSet = sequence.get(step);
            ds.get(varW[0][0]);
            systemStateSampleSet.mean(d -> d.getDataState().get(varW[0][0]));

            return null;

        };
    }


    FeedbackFunction<RandomGenerator, DataState, EvolutionSequence, List<DataStateUpdate>> TICK_FUNCTION = (rg, ds, sequence) -> List.of();

    DataState apply(RandomGenerator rg, DataState ds, EvolutionSequence sequence);
    */

}
