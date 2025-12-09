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

package it.unicam.quasylab.jspear.robtl;

import it.unicam.quasylab.jspear.EvolutionSequence;

/**
 * This functional interface represents an interpretation function of RobTL formulae.
 *
 * @param <T> formulae interpretation domain.
 */
@FunctionalInterface
public interface RobustnessFunction<T> {


    /**
     * Returns the evaluation of this function to the given <code>sequence</code> at the given <code>step</code>
     * and using the given <code>sampleSize</code>.
     *
     * @param sampleSize size of the sample set used to inter statistical values.
     * @param step computational step at which the formula is evaluated.
     * @param sequence evolution sequence to evaluate.
     * @return the evaluation of this function to the given <code>sequence</code> at the given <code>step</code>
     * and using the given <code>sampleSize</code>.
     */
    T eval(int sampleSize, int step, EvolutionSequence sequence);

}
