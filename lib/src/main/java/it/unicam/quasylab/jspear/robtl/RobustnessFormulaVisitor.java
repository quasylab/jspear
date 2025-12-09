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

/**
 * This interface offers the methods necessary for the evaluation of RobTL formulae
 * in the given interpretation domain (Boolean, or three-valued).
 *
 * @param <T> interpretation domain.
 */
public interface RobustnessFormulaVisitor<T> {

    RobustnessFunction<T> eval(RobustnessFormula formula);

    RobustnessFunction<T> evalAlways(AlwaysRobustnessFormula alwaysRobustnessFormula);

    RobustnessFunction<T> evalAtomic(AtomicRobustnessFormula atomicRobustnessFormula);

    RobustnessFunction<T> evalConjunction(ConjunctionRobustnessFormula conjunctionRobustnessFormula);

    RobustnessFunction<T> evalDisjunction(DisjunctionRobustnessFormula disjunctionRobustnessFormula);

    RobustnessFunction<T> evalEventually(EventuallyRobustnessFormula eventuallyRobustnessFormula);

    RobustnessFunction<T> evalFalse();

    RobustnessFunction<T> evalImplication(ImplicationRobustnessFormula implicationRobustnessFormula);

    RobustnessFunction<T> evalNegation(NegationRobustnessFormula negationRobustnessFormula);

    RobustnessFunction<T> evalTrue();

    RobustnessFunction<T> evalUntil(UntilRobustnessFormula untilRobustnessFormula);

}
