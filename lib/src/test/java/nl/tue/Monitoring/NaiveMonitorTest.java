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
import nl.tue.Monitoring.NaiveMonitorBuilder;

import org.apache.commons.math3.random.RandomGenerator;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.*;

class NaiveMonitorTest {

    final int x = 0;
    int seed = 0;
    // one variable x, evolution sequence composed of distributions s.t. x uniformly distributed in [0,1]
    // Target with tolerance 1 and mu following a uniform distribution over data states with values of x in [0,1]
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
    void targetTest1() {
        final int SAMPLE_SIZE = 10;

        EvolutionSequence sequence = getTestES1();

        DataStateFunction mu = (rg, ds) -> ds.apply(List.of(new DataStateUpdate(x, rg.nextDouble())));
        UDisTLFormula phi = new TargetDisTLFormula(mu, ds -> ds.get(x), 1.0);
        int timestep = 0;

        double semanticsEval = new DoubleSemanticsVisitor().eval((DisTLFormula) phi)
                .eval(SAMPLE_SIZE, 0, sequence);

        NaiveMonitorBuilder naiveMonitorBuilder = new NaiveMonitorBuilder(timestep, SAMPLE_SIZE, false);
        UDisTLMonitor<Double> m = naiveMonitorBuilder.build(phi);
        m.setRandomGeneratorSeed(seed);

        SampleSet<PerceivedSystemState> distribution = UDisTLMonitor.systemStatesToPerceivedSystemStates(sequence.get(0));

        double monitorEval = m.evalNext(distribution);
        assertEquals(semanticsEval, monitorEval);
    }
}

