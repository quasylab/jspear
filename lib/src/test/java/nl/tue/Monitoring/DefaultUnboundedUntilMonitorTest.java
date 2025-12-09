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

class DefaultUnboundedUntilMonitorTest {

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

        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(x, (1.0/(ds.get(t) + 1)))));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(idleController, environment, new DataState(new double[]{0, 1.0}));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        return new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);
    }


    // The distribution at time t is a dirac dist. around (0, -1.0) if t == 0, otherwise dirac dist. around (t, sin(t))
    static EvolutionSequence getTestES2(){
        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(x, Math.sin(ds.get(t) + 1))));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(idleController, environment, new DataState(new double[]{0, -1.0}));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        return new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);
    }


    // evolution sequence composed of distributions s.t. x uniformly distributed in [0,1]
    static EvolutionSequence getTestES3(){
        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(x, rg.nextDouble())));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(idleController, environment, new DataState(new double[]{0, rg.nextDouble()}));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        return new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);
    }



    static Stream<EvolutionSequence> getEvolutionSequences() {
        return Stream.of(
                getTestES1(),
                getTestES2(),
                getTestES3()
        );
    }

    @ParameterizedTest
    @MethodSource("getEvolutionSequences")
    void untilEvaluatesAt0(EvolutionSequence sequence) {
        // mu is a dirac dist around (0,0) and penalty fn P((t,x)) = ds(x)
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0)
                        ));
        DisTLFormula right = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);
        DisTLFormula left = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);

        int from = 0;
        int to = 5;

        DisTLFormula phi = new UntilDisTLFormula(left, from, to, right);
        int semanticsEvalTimestep = 0;

        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        double semanticsEval = semanticsEvaluator.eval(phi)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(phi, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);

//        System.out.println("Until test: evaluation at 0, ES "+sequence.toString());
        for (int i = from; i <= to+1; i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(i);

            OptionalDouble monitorEval = m.evalNext(observationSampleSet);
//            DataState perceivedSample =((PerceivedSystemState) observationSampleSet.stream().toArray()[0]).getDataState();
//            double[] leftEval = new double[i];
//            for (int j = 0; j < i; j++) {
//                leftEval[j] = semanticsEvaluator.eval(left).eval(SAMPLE_SIZE, j, sequence);
//            }
//            double rightEval = semanticsEvaluator.eval(right).eval(SAMPLE_SIZE, i, sequence);

//            System.out.printf("s_%d: dirac around (%.2f, %.4f), monitor output: %.4f, subformulae eval: left %.4f, right: ",
//                    i, perceivedSample.get(t), perceivedSample.get(x),
//                    monitorEval.isPresent() ? monitorEval.getAsDouble() : Double.NaN,
//                    rightEval);
//            System.out.println(Arrays.toString(leftEval));
        }

        OptionalDouble monitorEval = m.evalNext(emptySampleSet);
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }

    @ParameterizedTest
    @MethodSource("getEvolutionSequences")
    void untilEvaluatesAt3(EvolutionSequence sequence) {

        // mu is a dirac dist around (0,0) and penalty fn P((t,x)) = ds(x)
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0)
                ));
        DisTLFormula right = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);
        DisTLFormula left = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);

        int from = 2;
        int to = 7;

        DisTLFormula phi = new UntilDisTLFormula(left, from, to, right);
        int semanticsEvalTimestep = 3;

        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        double semanticsEval = semanticsEvaluator.eval(phi)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(phi, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);

        for (int i = 0; i <= semanticsEvalTimestep + to + 4; i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(i);
            OptionalDouble monitorEval = m.evalNext(observationSampleSet);

            // Monitoring values are defined for observation traces with length semanticsEvalTimestep + FES or greater
            if(i + 1 >= semanticsEvalTimestep + phi.getFES()){
                assertTrue(monitorEval.isPresent());
            } else {
                assertTrue(monitorEval.isEmpty());
            }

//            DataState perceivedSample =((PerceivedSystemState) observationSampleSet.stream().toArray()[0]).getDataState();
//            System.out.printf("s_%d: dirac around (%.2f, %.4f), monitor output: %.4f%n",
//                    i, perceivedSample.get(t), perceivedSample.get(x),
//                    monitorEval.isPresent() ? monitorEval.getAsDouble() : Double.NaN);


//            for (int tau = 0; tau <= i; tau++) {
//                double eval2 = semanticsEvaluator.eval(right).eval(SAMPLE_SIZE, tau, sequence);
//                double[] eval1 = new double[tau];
//                for (int tauprime = 0; tauprime < tau; tauprime++) {
//                    eval1[tauprime] = semanticsEvaluator.eval(left).eval(SAMPLE_SIZE, tauprime, sequence);
//                }
//                System.out.println("tau = "+tau);
//                System.out.printf("phi2 %.4f, phi1 ", eval2);
//                for (double v : eval1) {
//                    System.out.printf("%.4f ", v);
//                }
//                System.out.println();
//            }
        }

        OptionalDouble monitorEval = m.evalNext(emptySampleSet);
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }


    @Test
    void alwaysMonitorEvalEqualsSemanticsAt0(){
        int semanticsEvalTimestep = 0;
         DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0)
                ));
         DisTLFormula atomic = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);
        DisTLFormula formula = new UntilDisTLFormula(new TrueDisTLFormula(), 5, 11,
                new NegationDisTLFormula(atomic));
        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        EvolutionSequence sequence = getTestES2();
        double semanticsEval = semanticsEvaluator.eval(formula)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(formula, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);

        for (int i = 0; i < semanticsEvalTimestep + formula.getTimeHorizon().orElseThrow(); i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(i);
            OptionalDouble monitorEval = m.evalNext(observationSampleSet);
//            DataState perceivedSample = ((PerceivedSystemState) observationSampleSet.stream().toArray()[0]).getDataState();
//            System.out.printf("s_%d: dirac around (%.2f, %.4f), monitor output: %.4f%n", i, perceivedSample.get(t), perceivedSample.get(x), monitorEval.isPresent() ? monitorEval.getAsDouble() : Double.NaN);

//            for (int tau = 0; tau <= i; tau++) {
//                double eval2 = semanticsEvaluator.eval(atomic).eval(SAMPLE_SIZE, tau, sequence);
//                double[] eval1 = new double[tau];
//                for (int tauprime = 0; tauprime < tau; tauprime++) {
//                    eval1[tauprime] = semanticsEvaluator.eval(new TrueDisTLFormula()).eval(SAMPLE_SIZE, tauprime, sequence);
//                }
//                System.out.println("tau = " + tau);
//                System.out.printf("phi2 %.4f, phi1 ", eval2);
//                for (double v : eval1) {
//                    System.out.printf("%.4f ", v);
//                }
//                System.out.println();
//            }
        }



        OptionalDouble monitorEval = m.evalNext(sequence.getAsPerceivedSystemStates(semanticsEvalTimestep + formula.getTimeHorizon().orElseThrow()));
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }

    @Test
    void alwaysMonitorEvalEqualsSemanticsAt5(){
        int semanticsEvalTimestep = 5;

        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(t, 0),
                        new DataStateUpdate(x, 0.0)
                ));
        DisTLFormula atomic = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);
        DisTLFormula formula = new UntilDisTLFormula(new TrueDisTLFormula(), 5, 11,
                new NegationDisTLFormula(atomic));
        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        EvolutionSequence sequence = getTestES2();
        double semanticsEval = semanticsEvaluator.eval(formula)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(formula, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);

        for (int i = 0; i < semanticsEvalTimestep + formula.getTimeHorizon().orElseThrow(); i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(i);
            OptionalDouble monitorEval = m.evalNext(observationSampleSet);
//            DataState perceivedSample = ((PerceivedSystemState) observationSampleSet.stream().toArray()[0]).getDataState();
//            System.out.printf("s_%d: dirac around (%.2f, %.4f), monitor output: %.4f%n", i, perceivedSample.get(t), perceivedSample.get(x), monitorEval.isPresent() ? monitorEval.getAsDouble() : Double.NaN);

//            for (int tau = 0; tau <= i; tau++) {
//                double eval2 = semanticsEvaluator.eval(atomic).eval(SAMPLE_SIZE, tau, sequence);
//                double[] eval1 = new double[tau];
//                for (int tauprime = 0; tauprime < tau; tauprime++) {
//                    eval1[tauprime] = semanticsEvaluator.eval(new TrueDisTLFormula()).eval(SAMPLE_SIZE, tauprime, sequence);
//                }
//                System.out.println("tau = " + tau);
//                System.out.printf("phi2 %.4f, phi1 ", eval2);
//                for (double v : eval1) {
//                    System.out.printf("%.4f ", v);
//                }
//                System.out.println();
//            }
        }



        OptionalDouble monitorEval = m.evalNext(sequence.getAsPerceivedSystemStates(semanticsEvalTimestep + formula.getTimeHorizon().orElseThrow()));
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }


}

