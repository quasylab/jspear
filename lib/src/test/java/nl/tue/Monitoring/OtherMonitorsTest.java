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
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Function;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class OtherMonitorsTest {
    static int seed = 0;
    static final int SAMPLE_SIZE = 10;
    static final int ES_SAMPLE_SIZE = 10;
    static final SampleSet<PerceivedSystemState> emptySampleSet = new SampleSet<>();
    static Controller idleController;

    static final int t = 0;
    static final int x = 1;

    // mu is a dirac dist around (0,0) and penalty fn P((t,x)) = ds(x)
     static final DataStateFunction mu = (rg, ds) -> ds.apply(
            List.of(new DataStateUpdate(t, 0),
                    new DataStateUpdate(x, 0.0)
            ));
     static final DisTLFormula atomic = new TargetDisTLFormula(mu, ds -> ds.get(x), 0.0);

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


    static Stream<Object[]> getParameters(){
        List<EvolutionSequence> es = List.of(
                getTestES1(),
                getTestES2(),
                getTestES3()
        );
        List<DisTLFormula> formulae = List.of(
                new AlwaysDisTLFormula(atomic, 5, 11),
                new ConjunctionDisTLFormula(atomic, atomic),
                new DisjunctionDisTLFormula(atomic, atomic),
                new EventuallyDisTLFormula(atomic, 50, 52),
                new ImplicationDisTLFormula(atomic, atomic),
                new NegationDisTLFormula(atomic)
        );
        List<Integer> evaluationTimestep = List.of(0,5);
        return IntStream.range(0, es.size()*formulae.size()*evaluationTimestep.size())
                .mapToObj(i -> new Object[]{ es.get(i % es.size()), formulae.get(i % formulae.size()), evaluationTimestep.get(i % evaluationTimestep.size())});
        }

    @ParameterizedTest
    @MethodSource("getParameters")
    void monitorEvaluationEqualsSemantics(EvolutionSequence sequence, DisTLFormula formula, int semanticsEvalTimestep) {
        DoubleSemanticsVisitor semanticsEvaluator = new DoubleSemanticsVisitor();
        semanticsEvaluator.setRandomGeneratorSeed(seed);
        double semanticsEval = semanticsEvaluator.eval(formula)
                .eval(SAMPLE_SIZE, semanticsEvalTimestep, sequence);

        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(SAMPLE_SIZE, false);
        DefaultUDisTLMonitor m = defaultMonitorBuilder.build(formula, semanticsEvalTimestep);
        m.setRandomGeneratorSeed(seed);
//        System.out.println();
        for (int i = 0; i < semanticsEvalTimestep + formula.getTimeHorizon().orElseThrow(); i++) {
            SampleSet<PerceivedSystemState> observationSampleSet = sequence.getAsPerceivedSystemStates(i);

            OptionalDouble monitorEval = m.evalNext(observationSampleSet);
             if (i + 1 < semanticsEvalTimestep + formula.getFES()){
                assertTrue(monitorEval.isEmpty());
            } else {
                assertTrue(monitorEval.isPresent());
            }
//            DataState perceivedSample = ((PerceivedSystemState) observationSampleSet.stream().toArray()[0]).getDataState();
//            System.out.printf("s_%d: sample (%.2f, %.4f), monitor output: %.4f%n",
//                    i, perceivedSample.get(t), perceivedSample.get(x),
//                    monitorEval.isPresent() ? monitorEval.getAsDouble() : Double.NaN);
        }


        OptionalDouble monitorEval = m.evalNext(sequence.getAsPerceivedSystemStates(semanticsEvalTimestep + formula.getTimeHorizon().orElseThrow()));
        assertTrue(monitorEval.isPresent());
        assertEquals(semanticsEval, monitorEval.getAsDouble());
    }
}
