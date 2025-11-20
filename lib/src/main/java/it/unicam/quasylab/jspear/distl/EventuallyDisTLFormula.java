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

public final class EventuallyDisTLFormula implements DisTLFormula {

    private final UDisTLFormula arg;
    private final int from;
    private final int to;


    public EventuallyDisTLFormula(UDisTLFormula arg, int from, int to) {
        this.arg = arg;
        this.from = from;
        this.to = to;
    }

    @Override
    public <T> DisTLFunction<T> eval(DisTLFormulaVisitor<T> evaluator) {
        return evaluator.evalEventually(this);
    }

    public UDisTLFormula getArgument() {
        return this.arg;
    }

    public int getFrom() {
        return from;
    }

    public int getTo() {
        return to;
    }

    @Override
    public <T> T build(MonitorBuildingVisitor<T> visitor, int semanticsEvaluationTimestep) {
        return visitor.buildEventually(this, semanticsEvaluationTimestep);
    }

    @Override
    public int getFES() {
        return arg.getFES() + from;
    }

    @Override
    public OptionalInt getTimeHorizon() {
        OptionalInt argTimeHorizon = arg.getTimeHorizon();
        if(argTimeHorizon.isEmpty()){
            return OptionalInt.empty();
        } else {
            return OptionalInt.of(to + argTimeHorizon.getAsInt());
        }
    }
}
