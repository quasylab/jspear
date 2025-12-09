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
import org.apache.commons.math3.random.RandomGenerator;

/**
 * We use the classes implementing the interface to represent formulae in the Robustness Temporal Logic (RobTL).
 * The interface offers two methods to check whether a formula is satisfied:
 * one based on the classic Boolean semantics,
 * the other based on a three-valued semantics.
 */
public sealed interface RobustnessFormula permits
        AlwaysRobustnessFormula,
        AtomicRobustnessFormula,
        ConjunctionRobustnessFormula,
        DisjunctionRobustnessFormula,
        EventuallyRobustnessFormula,
        FalseRobustnessFormula,
        ImplicationRobustnessFormula,
        NegationRobustnessFormula,
        TrueRobustnessFormula,
        UntilRobustnessFormula {

    /**
     * Returns the evaluation of the formula according to a given interpretation function (Boolean, or three-valued).
     *
     * @param evaluator an interpretation function
     * @return the evaluation of this formula according to <code>evaluator</code>
     * @param <T> interpretation domain
     */
    <T> RobustnessFunction<T> eval(RobustnessFormulaVisitor<T> evaluator);

    /**
     * Returns the evaluation of a given formula according to classic Boolean semantics.
     *
     * @param formula a RobTL formula
     * @return the Boolean evaluation of <code>formula</code>.
     */
    static RobustnessFunction<Boolean> getBooleanEvaluationFunction(RobustnessFormula formula) {
        return formula.eval(new BooleanSemanticsVisitor());
    }

    /**
     * Returns the evaluation of a given formula according to three-valued semantics,
     * using default values for the bootstrap method in the evaluation of confidence intervals.
     *
     * @param formula a RobTL formula
     * @return the three-valued evaluation of <code>formula</code>.
     */
    static RobustnessFunction<TruthValues> getThreeValuedEvaluationFunction(RobustnessFormula formula) {
        return formula.eval(new ThreeValuedSemanticsVisitor());
    }

    /**
     * Returns the evaluation of a given formula according to three-valued semantics,
     * using custom values for the bootstrap method in the evaluation of confidence intervals.
     *
     * @param m number of repetitions for the boostrap method
     * @param z the quantile of the standard normal distribution corresponding to the desired coverage probability
     * @param formula a RobTL formula
     * @return the three-valued evaluation of <code>formula</code>.
     */
    static RobustnessFunction<TruthValues> getThreeValuedEvaluationFunction(RandomGenerator rg, int m, double z, RobustnessFormula formula) {
        return formula.eval(new ThreeValuedSemanticsVisitor(rg, m, z));
    }

}
