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
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.udistl.UDisTLFormula;
import nl.tue.Monitoring.Default.DefaultMonitorBuilder;
import nl.tue.Monitoring.Default.DefaultUDisTLMonitor;
import org.apache.commons.math3.random.RandomGenerator;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TargetMonitorTest {

    final int x = 0;
    int seed = 0;
    final int SAMPLE_SIZE = 10000;

    final SampleSet<PerceivedSystemState> emptySampleSet = new SampleSet<>();

    // one variable x, evolution sequence composed of distributions s.t. x uniformly distributed in [0,1]
    EvolutionSequence getTestES1(){
        final int ES_SAMPLE_SIZE = 10;
        final int NUMBER_OF_VARIABLES = 1;

        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Ctrl",
                Controller.doTick(registry.get("Ctrl"))
        );
        Controller controller = registry.reference("Ctrl");

        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(new DataStateUpdate(x, rg.nextDouble())));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(controller, environment, new DataState(NUMBER_OF_VARIABLES, i -> rg.nextDouble()));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        return new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);
    }




    @Test
    void targetEvaluatesAt0() {
        EvolutionSequence sequence = getTestES1();

        // Target with tolerance 1 and mu following a uniform distribution over data states with values of x in [0,1]
        DataStateFunction mu = (rg, ds) -> ds.apply(List.of(new DataStateUpdate(x, rg.nextDouble())));
        DisTLFormula phi = new TargetDisTLFormula(mu, ds -> ds.get(x), 1.0);
        int semanticsEvalTimestep = 0;

        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        double semanticsEval = semanticsEvaluator.eval(phi)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(phi, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);

        SampleSet<PerceivedSystemState> distribution =sequence.getAsPerceivedSystemStates(semanticsEvalTimestep);

        OptionalDouble monitorEval = m.evalNext(distribution);
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }

    // Target monitor is set up to monitor formula evaluated at timestep 1, thus for t=0 it returns undefined symbol
    @Test
    void targetReturnsUndefinedSymbolBeforeFES() {
        DataStateFunction mu = (rg, ds) -> ds.apply(List.of(new DataStateUpdate(x, rg.nextDouble())));
        UDisTLFormula phi = new TargetDisTLFormula(mu, ds -> ds.get(x), 1.0);
        int semanticsEvalTimestep = 1;

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(phi, semanticsEvalTimestep);

        assertTrue(m.evalNext(emptySampleSet).isEmpty());
    }

    // Target monitor is set up to monitor formula evaluated at timestep 1, so monitor returns semantics evaluation when t = 1
    @Test
    void targetEvaluatesAt1() {
        EvolutionSequence sequence = getTestES1();

        DataStateFunction mu = (rg, ds) -> ds.apply(List.of(new DataStateUpdate(x, rg.nextDouble())));
        DisTLFormula phi = new TargetDisTLFormula(mu, ds -> ds.get(x), 1.0);
        int semanticsEvalTimestep = 1;

        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        double semanticsEval = semanticsEvaluator.eval(phi)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(phi, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);

        m.evalNext(emptySampleSet);

        SampleSet<PerceivedSystemState> distribution = sequence.getAsPerceivedSystemStates(semanticsEvalTimestep);
        OptionalDouble monitorEval = m.evalNext(distribution);
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }

    final int t = 0;
    final int y = 1;
    // Two variables. Evolution sequence defined as follows:
    // The distribution at time t is a dirac dist. around (0, 1.0) if t == 0, otherwise dirac dist. around (t, 1/t)
    EvolutionSequence getTestES2(){
        final int ES_SAMPLE_SIZE = 10;
        final int NUMBER_OF_VARIABLES = 2;

        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Ctrl",
                Controller.doTick(registry.get("Ctrl"))
        );
        Controller controller = registry.reference("Ctrl");

        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(y, (1.0/(ds.get(t) + 1)))));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(controller, environment, new DataState(NUMBER_OF_VARIABLES, i -> 1.0));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        return new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);
    }

    @Test
        // Target with tolerance 0 and mu dirac distribution around (0,0.0)
    void targetEvaluatesCorrectly2() {
        EvolutionSequence sequence = getTestES2();

        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(y, 0.0)
                ));
        DisTLFormula phi = new TargetDisTLFormula(mu, ds -> ds.get(y), 0);
        int semanticsEvalTimestep = 2;

        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        double semanticsEval = semanticsEvaluator.eval(phi)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(phi, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);

        m.evalNext(emptySampleSet);
        m.evalNext(emptySampleSet);

        SampleSet<PerceivedSystemState> distribution = sequence.getAsPerceivedSystemStates(semanticsEvalTimestep);

        OptionalDouble monitorEval = m.evalNext(distribution);
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }


}

