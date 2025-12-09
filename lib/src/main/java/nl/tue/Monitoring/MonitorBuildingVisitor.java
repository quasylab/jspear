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

public interface MonitorBuildingVisitor<T> {
    T build(UDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildAlways(AlwaysDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildBrink(BrinkDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildConjunction(ConjunctionDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildDisjunction(DisjunctionDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildEventually(EventuallyDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildFalse(FalseDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildImplication(ImplicationDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildNegation(NegationDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildTarget(TargetDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildTrue(TrueDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildUntil(UntilDisTLFormula formula, int semanticsEvaluationTimestep);

    T buildUnboundedUntil(UnboundedUntiluDisTLFormula formula, int semanticsEvaluationTimestep);


}
