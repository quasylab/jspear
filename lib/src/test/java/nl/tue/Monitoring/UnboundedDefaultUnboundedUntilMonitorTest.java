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
import it.unicam.quasylab.jspear.distl.*;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateFunction;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.udistl.UDisTLFormula;
import it.unicam.quasylab.jspear.udistl.UnboundedUntiluDisTLFormula;
import nl.tue.Monitoring.Default.DefaultMonitorBuilder;
import nl.tue.Monitoring.Default.DefaultUDisTLMonitor;
import org.apache.commons.math3.random.RandomGenerator;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
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
    static final int y = 2;

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

    @Test
    void monotononicityCounterexample() {
        Function<Double, Double> sine = t -> (Math.sin(t/3.0)+1)/2.0;  // function that goes /\/\/\/\/ with period around 10
        Function<Double, Double> sineShifted = t -> (Math.sin((t/3.0)+Math.PI)+1)/2.0;  // function that goes /\/\/\/\/ with period around 10
        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
//                new DataStateUpdate(x, ds.get(t) % 2 == 0 ? 1.5- (1/Math.sqrt(Math.log(ds.get(t)+Math.exp(1)))): (1/Math.sqrt(Math.log(ds.get(t)+Math.exp(1)))) - 0.5)));
                new DataStateUpdate(x, sine.apply(ds.get(t))),
                new DataStateUpdate(y, sineShifted.apply(ds.get(t)))));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(idleController, environment, new DataState(new double[]{5, sine.apply(5.0), sineShifted.apply(5.0)}));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        EvolutionSequence sequence = new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);

        int TEST_LIMIT = 10;
        // mu is a dirac dist around (0,0) and penalty fn P((t,x)) = ds(x)
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0),
                        new DataStateUpdate(y, 0.0)
                ));
        DisTLFormula atomicUp = new TargetDisTLFormula(mu, ds -> ds.get(y), 0.0);
        DisTLFormula atomicDown = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);
        DisTLFormula goesUp = new AlwaysDisTLFormula(atomicUp,0,8);
        DisTLFormula goesDown = new EventuallyDisTLFormula(atomicDown,0,8);
        DisTLFormula goesUpAndDown = new ConjunctionDisTLFormula(goesDown, goesUp);

        int semanticsEvalTimestep = 0;
        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);

        DefaultUDisTLMonitor mAlways = defaultMonitorBuilder.build(goesUp, semanticsEvalTimestep);
        DefaultUDisTLMonitor mEvent = defaultMonitorBuilder.build(goesDown, semanticsEvalTimestep);
        DefaultUDisTLMonitor mCon = defaultMonitorBuilder.build(goesUpAndDown, semanticsEvalTimestep);
        mAlways.setRandomGeneratorSeed(seed);
        mEvent.setRandomGeneratorSeed(seed);
        mCon.setRandomGeneratorSeed(seed);
        int from = 0;
        for (int i = 0; i < TEST_LIMIT; i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(i);
            DataState sample = observationSampleSet.stream().toList().get(1).getDataState();

            OptionalDouble mEvalAlways = mAlways.evalNext(observationSampleSet);
            OptionalDouble mEvalEvent = mEvent.evalNext(observationSampleSet);
            OptionalDouble mEvalCon = mCon.evalNext(observationSampleSet);

            System.out.printf("t=%d, x=%.4f, y=%.4f, mon output: always=%.4f, event=%.4f con=%.4f %n", i, sample.get(x), sample.get(y), mEvalAlways.isPresent() ? mEvalAlways.getAsDouble() : 9.0, mEvalEvent.isPresent() ? mEvalEvent.getAsDouble() : 9.0, mEvalCon.isPresent() ? mEvalCon.getAsDouble() : 9.0);


        }
    }

    @Test
    void tarsCounterexample() {
        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(x, ds.get(t) % 2 == 0 ? 1 : 0)));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(idleController, environment, new DataState(new double[]{0.0, 0.0}));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        EvolutionSequence sequence = new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);

        int TEST_LIMIT = 1000;
        // mu is a dirac dist around (0,0) and penalty fn P((t,x)) = ds(x)
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0)
                ));
        DisTLFormula atomic = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);
        DisTLFormula T = new TrueDisTLFormula();
        UDisTLFormula counterex = new UnboundedUntiluDisTLFormula(T, new NegationDisTLFormula(new UnboundedUntiluDisTLFormula(T, atomic)));


        int semanticsEvalTimestep = 0;
        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);

        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(counterex, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);
        int from = 0;
        double previous = -9.0;
        for (int i = 0; i < TEST_LIMIT; i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(i);
            DataState sample = observationSampleSet.stream().toList().get(1).getDataState();

            OptionalDouble mEval = m.evalNext(observationSampleSet);
            if (mEval.isPresent()){
                double current = mEval.getAsDouble();
                if(current == previous){
                    System.out.println("Monotonicity encountered at t="+i);
                }
                previous = current;
            }
//            System.out.printf("t=%d, x=%.4f, mon output:%.4f %n", i, sample.get(x), mEval.isPresent() ? mEval.getAsDouble() : 9.0);


        }
    }

    @Test
    void tarsCounterexampleWBounded() {
        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(x, ds.get(t) % 2 == 0 ? 1 : 0)));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(idleController, environment, new DataState(new double[]{0.0, 0.0}));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        EvolutionSequence sequence = new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);

        int TEST_LIMIT = 1000;
        // mu is a dirac dist around (0,0) and penalty fn P((t,x)) = ds(x)
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0)
                ));
        DisTLFormula atomic = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);
        DisTLFormula T = new TrueDisTLFormula();
        UDisTLFormula counterex = new UnboundedUntiluDisTLFormula(T, new NegationDisTLFormula(new UntilDisTLFormula(T, 0,2, atomic)));


        int semanticsEvalTimestep = 0;
        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);

        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(counterex, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);
        int from = 0;
        double previous = -9.0;
        for (int i = 0; i < TEST_LIMIT; i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(i);
            DataState sample = observationSampleSet.stream().toList().get(1).getDataState();

            OptionalDouble mEval = m.evalNext(observationSampleSet);
            if (mEval.isPresent()){
                double current = mEval.getAsDouble();
                if(current == previous){
                    System.out.println("Monotonicity encountered at t="+i);
                }
                previous = current;
            }
            System.out.printf("t=%d, x=%.4f, mon output:%.4f %n", i, sample.get(x), mEval.isPresent() ? mEval.getAsDouble() : 9.0);


        }
    }
}

