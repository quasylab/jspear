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
import it.unicam.quasylab.jspear.udistl.UDisTLFormula;
import it.unicam.quasylab.jspear.udistl.UnboundedUntiluDisTLFormula;
import nl.tue.Monitoring.Default.DefaultMonitorBuilder;
import nl.tue.Monitoring.Default.DefaultUDisTLMonitor;
import org.apache.commons.math3.random.RandomGenerator;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Function;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class UnboundedDefaultUnboundedUntilMonitorTest {

    static int seed = 0;
    static final int SAMPLE_SIZE = 10;
    static final int ES_SAMPLE_SIZE = 10;
    static final SampleSet<PerceivedSystemState> emptySampleSet = new SampleSet<>();
    static Controller idleController;

    static final int t = 0;
    static final int x = 1;

    @BeforeAll
    static void setup(){
        final ControllerRegistry registry = new ControllerRegistry();
        registry.set("Ctrl",
                Controller.doTick(registry.get("Ctrl"))
        );
        idleController = registry.reference("Ctrl");
    }


    // Two variables. Evolution sequence defined as follows:
    // The distribution at time t is a dirac dist. around (0, 1.0) if t == 0, otherwise dirac dist. around (t, 1/t)
    static EvolutionSequence getTestES1(){
        int NUMBER_OF_VARIABLES = 2;

        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(x, (1.0/(ds.get(t) + 1)))));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(idleController, environment, new DataState(new double[]{0, 1.0}));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        return new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);
    }

    // Two variables. Evolution sequence defined as follows:
    // The distribution at time t is a dirac dist. around (0, -1.0) if t == 0, otherwise dirac dist. around (t, sin(t))
    static EvolutionSequence getTestES2(){
        int NUMBER_OF_VARIABLES = 2;

        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(x, Math.sin(ds.get(t) + 1))));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(idleController, environment, new DataState(new double[]{0, -1.0}));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        return new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);
    }


    static Stream<EvolutionSequence> getEvolutionSequences() {
        return Stream.of(
                getTestES1(),
                getTestES2()
        );
    }

    @ParameterizedTest
    @MethodSource("getEvolutionSequences")
    void truncatedUnboundedUntilMonitorEqualsUntilMonitor(EvolutionSequence sequence) {
        int TEST_LIMIT = 30;
        // mu is a dirac dist around (0,0) and penalty fn P((t,x)) = ds(x)
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0)
                        ));
        DisTLFormula right = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);
        DisTLFormula left = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);

        int semanticsEvalTimestep = 0;
        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);

        UDisTLFormula phi = new UnboundedUntiluDisTLFormula(left, right);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(phi, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);
        int from = 0;
        for (int i = 0; i < TEST_LIMIT; i++) {
            DisTLFormula truncatedPhi = new UntilDisTLFormula(left, from, from+i+1, right);

            DefaultUDisTLMonitor mTruncated = defaultMonitorBuilder.build(truncatedPhi, semanticsEvalTimestep);
            mTruncated.setRandomGeneratorSeed(seed);
            for (int j = 0; j < i; j++) {
                SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(j);
                mTruncated.evalNext(observationSampleSet);
            }
            SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(i);
            OptionalDouble truncatedMonitorEval = mTruncated.evalNext(observationSampleSet);

            OptionalDouble monitorEval = m.evalNext(observationSampleSet);

            assertEquals(truncatedMonitorEval.isPresent(), monitorEval.isPresent());

            if(truncatedMonitorEval.isPresent()) {
                assertTrue(monitorEval.isPresent());
                assertEquals(truncatedMonitorEval.getAsDouble(), monitorEval.getAsDouble());
            }

        }

    }



}

