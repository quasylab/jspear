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

package it.unicam.quasylab.jspear.distl;

import it.unicam.quasylab.jspear.udistl.UDisTLFormula;
import nl.tue.Monitoring.MonitorBuildingVisitor;

import java.util.OptionalInt;

public final class ConjunctionDisTLFormula implements DisTLFormula {

    private final UDisTLFormula leftFormula;
    private final UDisTLFormula rightFormula;

    public ConjunctionDisTLFormula(UDisTLFormula leftFormula, UDisTLFormula rightFormula) {
        this.leftFormula = leftFormula;
        this.rightFormula = rightFormula;
    }


    @Override
    public <T> DisTLFunction<T> eval(DisTLFormulaVisitor<T> evaluator) {
        return evaluator.evalConjunction(this);
    }

    public UDisTLFormula getLeftFormula() {
        return leftFormula;
    }

    public UDisTLFormula getRightFormula() {
        return rightFormula;
    }

    @Override
    public <T> T build(MonitorBuildingVisitor<T> visitor, int semanticsEvaluationTimestep) {
        return visitor.buildConjunction(this, semanticsEvaluationTimestep);
    }

    @Override
    public int getFES() {
        return Math.max(leftFormula.getFES(), rightFormula.getFES());
    }

    @Override
    public OptionalInt getTimeHorizon() {
        OptionalInt l = leftFormula.getTimeHorizon();
        OptionalInt r = rightFormula.getTimeHorizon();
        if(l.isEmpty() || r.isEmpty()){
            return OptionalInt.empty();
        } else {
            return OptionalInt.of(Math.max(l.getAsInt(), r.getAsInt()));
        }
    }
}
