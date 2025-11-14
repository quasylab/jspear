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

package nl.tue.Monitoring;

import it.unicam.quasylab.jspear.distl.*;
import it.unicam.quasylab.jspear.udistl.UDisTLFormula;
import it.unicam.quasylab.jspear.udistl.UnboundedUntiluDisTLFormula;

import java.util.OptionalDouble;

public class DefaultMonitorBuilder implements MonitorBuildingVisitor<UDisTLMonitor<OptionalDouble>> {

    int sampleSize;
    boolean parallel;

    public DefaultMonitorBuilder(int sampleSize, boolean parallel) {
        this.sampleSize = sampleSize;
        this.parallel = parallel;
    }

    @Override
    public UDisTLMonitor<OptionalDouble> build(UDisTLFormula formula, int semanticsEvaluationTimestep) {
        return formula.build(this, semanticsEvaluationTimestep);
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildAlways(AlwaysDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildBrink(BrinkDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildConjunction(ConjunctionDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildDisjunction(DisjunctionDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildEventually(EventuallyDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildFalse(FalseDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildImplication(ImplicationDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildNegation(NegationDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildTarget(TargetDisTLFormula formula, int formulaEvalTimestep) {
        return new DefaultTargetMonitor(formula, formulaEvalTimestep, sampleSize, parallel);
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildTrue(TrueDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }


    @Override
    public UDisTLMonitor<OptionalDouble> buildUnboundedUntil(UnboundedUntiluDisTLFormula formula, int formulaEvalTimestep) {
        throw new UnsupportedOperationException("TODO");
    }

    @Override
    public UDisTLMonitor<OptionalDouble> buildUntil(UntilDisTLFormula formula, int formulaEvalTimestep) {
        return new DefaultUntilMonitor(formula, formulaEvalTimestep, sampleSize, parallel);
    }
}
