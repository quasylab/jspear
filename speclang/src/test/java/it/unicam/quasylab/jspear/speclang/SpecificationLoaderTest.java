/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
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

package it.unicam.quasylab.jspear.speclang;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;
import it.unicam.quasylab.jspear.robtl.TruthValues;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

class SpecificationLoaderTest {

    public static final String RANDOM_WALK = "./RandomWalk.jspec";
    public static final String ENGINE = "./Engine.jspec";
    public static final String VEHICLE = "./Vehicle.jspec";

    @Test
    @Disabled
    void loadRandomWalkSpecification() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(RANDOM_WALK)).openStream());
        assertNotNull(spec);
    }
    @Test
    @Disabled
    void loadEngineSpecification() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(ENGINE)).openStream());
        assertNotNull(spec);
    }

    @Test
    @Disabled
    void loadVehicleSpecification() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        assertNotNull(spec);
    }

    @Test
    @Disabled
    void loadVehicleSystem() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        ControlledSystem system = spec.getSystem();
        assertNotNull(system);
    }

    @Test
    @Disabled
    void simulateVehicleSystem() throws IOException {
            SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        assertNotNull(spec.getSamplesAt(50));
    }

    @Test
    @Disabled
    void stepVehicleSystem() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        ControlledSystem system = spec.getSystem();
        assertNotNull(system.sampleNext(new DefaultRandomGenerator()));
    }


    @Test
    @Disabled
    void loadVehicleProperty() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        RobustnessFormula formula = spec.getFormula("phi_crash_speed");
        assertNotNull(formula);
    }

    @Test
    @Disabled
    void vehiclePropertyBooleanCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(50);
        assertFalse(spec.evalBooleanSemantic("phi_crash_speed", 1, 5));
    }

    @Test
    @Disabled
    void vehiclePropertyThreeValuedCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(50);
        spec.setM(50);
        spec.setZ(1.96);
        assertEquals(TruthValues.FALSE, spec.evalThreeValuedSemantic("phi_crash_speed", 1, 5));
    }


    @Test
    @Disabled
    void loadSlowProperty() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        RobustnessFormula formula = spec.getFormula("phi_slow");
        assertNotNull(formula);
    }

    @Test
    void loadSample() throws IOException{
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setM(1);
        ControlledSystem system = spec.getSystem();
        EvolutionSequence sequence = spec.getSequence();
        DataStateExpression f = spec.getPenalty("rho_crash_speed");
        SampleSet sample = spec.getSamplesAt(300);
        double[] gino = sample.evalPenaltyFunction(f);
        for (int i = 0; i<gino.length; i++){
            System.out.println(gino[i]);
        }
        double[] data = SystemState.sample(new DefaultRandomGenerator(), f, system, 300, 1);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d> %f\n", i, data[i]);
        }
    }

    @Test
    void vehicleSlowThreeValuedCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        assertEquals(TruthValues.UNKNOWN, spec.evalThreeValuedSemantic("phi_slow", 60, 0));
    }

    @Test
    @Disabled
    void vehicleEvalPenaltyFunction() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        DataStateExpression f = spec.getPenalty("rho_crash");
        assertEquals(0, f.eval(spec.getSystem().getDataState()));
    }

    @Test
    @Disabled
    void vehicleSlowThreeValuedCheckTen() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.TRUE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_slow", 60, 0,100,10));
    }

    @Test
    void testApplyPerturbation() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        assertNotNull(spec.applyPerturbation("p_ItSlow", 0, 60, 400));
    }

    @Test
    void testOnPerturbation() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        EvolutionSequence sequence = spec.applyPerturbation("p_ItSlow", 0, 60, 150);
        DataStateExpression f = spec.getPenalty("rho_crash");
        SampleSet sample0 = sequence.get(350);
        double[] gino0 = sample0.evalPenaltyFunction(f);
        for (int i = 0; i<gino0.length; i++){
            System.out.println(gino0[i]);
        }
        SampleSet sample1 = sequence.get(400);
        double[] gino1 = sample1.evalPenaltyFunction(f);
        for (int i = 0; i<gino1.length; i++){
            System.out.println(gino1[i]);
        }
        SampleSet sample5 = sequence.get(450);
        double[] gino5 = sample5.evalPenaltyFunction(f);
        for (int i = 0; i<gino5.length; i++) {
            System.out.println(gino5[i]);
        }
    }

    @Test
    void testEvalDistance() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        assertTrue(spec.evalDistanceExpression("exp_crash", "p_ItSlow", 350, 60)>0);
    }

    @Test
    @Disabled
    void vehicleSlowThreeValuedCheckThirty() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.TRUE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_slow", 60, 0,300,30));
    }

    @Test
    @Disabled
    void vehicleSlowThreeValuedCheckFifty() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.TRUE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_slow", 60, 0,500,50));
    }

    @Test
    void vehicleSlowOffset03() throws IOException{
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        assertEquals(TruthValues.UNKNOWN, spec.evalThreeValuedSemantic("phi_slow", 60,50));
    }
    @Test
    void vehicleCombOffset03() throws IOException{
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        assertEquals(TruthValues.UNKNOWN, spec.evalThreeValuedSemantic("phi_comb", 60,10));
    }


    @Test
    void vehicleCombThreeValuedCheckTen() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.TRUE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_comb", 60, 0,100,10));
    }

    @Test
    @Disabled
    void vehicleCombThreeValuedCheckThirty() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.TRUE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_comb", 60, 0,300,30));
    }

    @Test
    @Disabled
    void vehicleCombThreeValuedCheckFifty() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.TRUE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_comb", 60, 0,500,50));
    }

    @Test
    @Disabled
    void vehicleCrashThreeValuedCheckTen() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.UNKNOWN);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_crash_speed", 60, 0,100,10));
    }

    @Test
    @Disabled
    void vehicleCrashThreeValuedCheckFifty() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setSize(1);
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = {TruthValues.UNKNOWN,TruthValues.UNKNOWN,TruthValues.UNKNOWN,TruthValues.UNKNOWN,TruthValues.UNKNOWN,TruthValues.UNKNOWN,TruthValues.UNKNOWN,TruthValues.UNKNOWN,TruthValues.TRUE,TruthValues.TRUE};
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_crash_speed", 60, 0,500,50));
    }

}