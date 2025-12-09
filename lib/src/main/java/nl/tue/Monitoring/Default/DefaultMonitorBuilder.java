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

package nl.tue.Monitoring.Default;

import it.unicam.quasylab.jspear.distl.*;
import it.unicam.quasylab.jspear.udistl.UDisTLFormula;
import it.unicam.quasylab.jspear.udistl.UnboundedUntiluDisTLFormula;
import nl.tue.Monitoring.MonitorBuildingVisitor;

import java.util.OptionalDouble;

public class DefaultMonitorBuilder implements MonitorBuildingVisitor<DefaultUDisTLMonitor> {

    int sampleSize;
    boolean parallel;


    public DefaultMonitorBuilder(int sampleSize, boolean parallel) {
        this.sampleSize = sampleSize;
        this.parallel = parallel;
    }

    /**
     * Builds a monitor for the given UDisTL formula.
     *
     * <p>The returned {@link DefaultUDisTLMonitor} consumes sample sets step by step through
     * {@code evalNext}, and produces an {@link OptionalDouble} as monitoring output.
     * The monitor internally tracks how many sample sets it has received, i.e. it is stateful.</p>
     *
     * <p>If an object {@code formula} of type {@link UDisTLFormula} represents a
     * (DisTL or UDisTL) formula {@code φ}, then this method constructs and returns
     * a monitor that implements the formal definition of monitor {@code m[φ]}.</p>
     *
     * @param formula the UDisTL formula to be monitored.
     * @param semanticsEvaluationTimestep the step at which the semantic evaluation should begin.
     * @return a stateful monitor that evaluates the given formula over incoming samples.
     */
    @Override
    public DefaultUDisTLMonitor build(UDisTLFormula formula, int semanticsEvaluationTimestep) {
        return formula.build(this, semanticsEvaluationTimestep);
    }

    public DefaultUDisTLMonitor build(UDisTLFormula formula) {
        return formula.build(this, 0);
    }

    @Override
    public DefaultUDisTLMonitor buildAlways(AlwaysDisTLFormula formula, int formulaEvalTimestep) {
        UDisTLFormula equivalent = new NegationDisTLFormula(
                new UntilDisTLFormula(new TrueDisTLFormula(),
                        formula.getFrom(), formula.getTo(),
                new NegationDisTLFormula(formula.getArgument())));
        return build(equivalent, formulaEvalTimestep);
    }

    @Override
    public DefaultUDisTLMonitor buildBrink(BrinkDisTLFormula formula, int formulaEvalTimestep) {
       return new BrinkMonitor(formula, formulaEvalTimestep, sampleSize, parallel);
    }

    @Override
    public DefaultUDisTLMonitor buildConjunction(ConjunctionDisTLFormula formula, int formulaEvalTimestep) {
        return new ConjunctionMonitor(formula, formulaEvalTimestep, sampleSize, parallel);
    }

    @Override
    public DefaultUDisTLMonitor buildDisjunction(DisjunctionDisTLFormula formula, int formulaEvalTimestep) {
        return new DisjunctionMonitor(formula, formulaEvalTimestep, sampleSize, parallel);
    }

    @Override
    public DefaultUDisTLMonitor buildEventually(EventuallyDisTLFormula formula, int formulaEvalTimestep) {
        UDisTLFormula equivalent = new UntilDisTLFormula(new TrueDisTLFormula(), formula.getFrom(), formula.getTo(),
                formula.getArgument());
        return build(equivalent, formulaEvalTimestep);
    }

    @Override
    public DefaultUDisTLMonitor buildFalse(FalseDisTLFormula formula, int formulaEvalTimestep) {
        UDisTLFormula equivalent = new NegationDisTLFormula(new TrueDisTLFormula());
        return build(equivalent, formulaEvalTimestep);
    }

    @Override
    public DefaultUDisTLMonitor buildImplication(ImplicationDisTLFormula formula, int formulaEvalTimestep) {
        UDisTLFormula equivalent = new DisjunctionDisTLFormula(new NegationDisTLFormula(formula.getLeftFormula()), formula.getRightFormula());
        return build(equivalent, formulaEvalTimestep);
    }

    @Override
    public DefaultUDisTLMonitor buildNegation(NegationDisTLFormula formula, int formulaEvalTimestep) {
        return new NegationMonitor(formula, formulaEvalTimestep, sampleSize, parallel);
    }

    @Override
    public DefaultUDisTLMonitor buildTarget(TargetDisTLFormula formula, int formulaEvalTimestep) {
        return new TargetMonitor(formula, formulaEvalTimestep, sampleSize, parallel);
    }

    @Override
    public DefaultUDisTLMonitor buildTrue(TrueDisTLFormula formula, int formulaEvalTimestep) {
        return new TrueMonitor();
    }


    @Override
    public DefaultUDisTLMonitor buildUnboundedUntil(UnboundedUntiluDisTLFormula formula, int formulaEvalTimestep) {
       return new UnboundedUntilMonitor(formula, formulaEvalTimestep, sampleSize, parallel);
    }

    @Override
    public DefaultUDisTLMonitor buildUntil(UntilDisTLFormula formula, int formulaEvalTimestep) {
        return new UntilMonitor(formula, formulaEvalTimestep, sampleSize, parallel);
    }
}
