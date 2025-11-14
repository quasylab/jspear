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

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.distl.DisTLFormula;
import it.unicam.quasylab.jspear.distl.DoubleSemanticsVisitor;
import it.unicam.quasylab.jspear.distl.TargetDisTLFormula;
import it.unicam.quasylab.jspear.distl.UntilDisTLFormula;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class UntilMonitorTest {

    final int t = 0;
    final int x = 1;
    int seed = 0;
    final int SAMPLE_SIZE = 10;

    final SampleSet<PerceivedSystemState> emptySampleSet = new SampleSet<>();

    // Two variables. Evolution sequence defined as follows:
    // The distribution at time t is a dirac dist. around (0, 1.0) if t == 0, otherwise dirac dist. around (t, 1/t)
    EvolutionSequence getTestES1(){
        final int ES_SAMPLE_SIZE = 10;
        final int NUMBER_OF_VARIABLES = 2;

        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Ctrl",
                Controller.doTick(registry.get("Ctrl"))
        );
        Controller controller = registry.reference("Ctrl");

        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(x, (1.0/(ds.get(t) + 1)))));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(controller, environment, new DataState(NUMBER_OF_VARIABLES, i -> 1.0));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        return new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);
    }

    @Test
    void untilsEventuallyEvaluatesAt0() {
        EvolutionSequence sequence = getTestES1();
        // mu is a dirac dist around (0,0) and penalty fn P((t,x)) = ds(x)
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0)
                        ));
        DisTLFormula right = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);

        // I have not implemented the monitor for DisTL's True, but this works the same:
        DisTLFormula TRUE = new TargetDisTLFormula(mu, ds -> ds.get(x), 10.0);

        int from = 0;
        int to = 5;

        DisTLFormula phi = new UntilDisTLFormula(TRUE, from, to, right);
        int semanticsEvalTimestep = 0;

        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        double semanticsEval = semanticsEvaluator.eval(phi)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        UDisTLMonitor<OptionalDouble> m = defaultMonitorBuilder.build(phi, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);

        for (int i = from; i <= to+1; i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = UDisTLMonitor.systemStatesToPerceivedSystemStates(sequence.get(i));
            OptionalDouble monitorEval = m.evalNext(observationSampleSet);
            DataState percievedSample =((PerceivedSystemState) observationSampleSet.stream().toArray()[0]).getDataState();
            System.out.printf("s_%d: dirac around (%.2f, %.4f), monitor output: %.4f%n", i, percievedSample.get(t), percievedSample.get(x), monitorEval.isPresent() ? monitorEval.getAsDouble() : Double.NaN);
        }

        OptionalDouble monitorEval = m.evalNext(emptySampleSet);
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }

    @Test
    void untilsEventuallyEvaluatesAt3() {
        EvolutionSequence sequence = getTestES1();
        // mu is a dirac dist around (0,0) and penalty fn P((t,x)) = ds(x)
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0)
                ));
        DisTLFormula right = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);

        // I have not implemented the monitor for DisTL's True, but this works the same:
        DisTLFormula TRUE = new TargetDisTLFormula(mu, ds -> ds.get(x), 10.0);

        int from = 2;
        int to = 7;

        DisTLFormula phi = new UntilDisTLFormula(TRUE, from, to, right);
        int semanticsEvalTimestep = 3;

        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        double semanticsEval = semanticsEvaluator.eval(phi)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        UDisTLMonitor<OptionalDouble> m = defaultMonitorBuilder.build(phi, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);

        for (int i = 0; i <= semanticsEvalTimestep + to + 3; i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = UDisTLMonitor.systemStatesToPerceivedSystemStates(sequence.get(i));
            OptionalDouble monitorEval = m.evalNext(observationSampleSet);
            DataState percievedSample =((PerceivedSystemState) observationSampleSet.stream().toArray()[0]).getDataState();
            System.out.printf("s_%d: dirac around (%.2f, %.4f), monitor output: %.4f%n", i, percievedSample.get(t), percievedSample.get(x), monitorEval.isPresent() ? monitorEval.getAsDouble() : Double.NaN);
        }

        OptionalDouble monitorEval = m.evalNext(emptySampleSet);
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }

}

