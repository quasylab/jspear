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

package it.unicam.quasylab.jspear.udistl;

import it.unicam.quasylab.jspear.distl.DisTLFormulaVisitor;
import it.unicam.quasylab.jspear.distl.DisTLFunction;
import nl.tue.Monitoring.MonitorBuildingVisitor;

import java.util.OptionalInt;

public class UnboundedUntiluDisTLFormula implements UDisTLFormula {

    private final UDisTLFormula rightFormula;
    private final UDisTLFormula leftFormula;

    public UnboundedUntiluDisTLFormula(UDisTLFormula leftFormula, UDisTLFormula rightFormula) {
        this.leftFormula = leftFormula;
        this.rightFormula = rightFormula;
    }

    public UDisTLFormula getRightFormula() {
        return rightFormula;
    }

    public UDisTLFormula getLeftFormula() {
        return leftFormula;
    }

    @Override
    public <T> T build(MonitorBuildingVisitor<T> visitor, int semanticsEvaluationTimestep) {
        return visitor.buildUnboundedUntil(this, semanticsEvaluationTimestep);
    }

    @Override
    public int getFES() {
        return Math.max(leftFormula.getFES(), rightFormula.getFES());
    }

    @Override
    public OptionalInt getTimeHorizon() {
        return OptionalInt.empty();
    }

    @Override
    public <T> DisTLFunction<T> eval(DisTLFormulaVisitor<T> evaluator) {
        throw new UnsupportedOperationException("Semantic evaluation of unbounded until is not formally defined");
    }

}
