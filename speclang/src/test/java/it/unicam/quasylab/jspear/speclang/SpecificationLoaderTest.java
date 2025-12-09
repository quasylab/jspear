/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *              Copyright (C) 2023.
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
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;
import it.unicam.quasylab.jspear.robtl.TruthValues;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

@Disabled
class SpecificationLoaderTest {

    public static final String RANDOM_WALK = "./RandomWalk.jspec";
    public static final String ENGINE = "./Engine.jspec";
    public static final String VEHICLE = "two_vehicles.jspec";
    public static final String SINGLE_VEHICLE = "./single_vehicle.jspec";


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
    void EngineFormulaBoolCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(ENGINE)).openStream());
        assertTrue(spec.evalBooleanSemantic("phi", 100, 0));
    }

    @Test
    void EngineFormulaThreeValuedCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(ENGINE)).openStream());
        spec.setSize(100);
        spec.setM(50);
        spec.setZ(1.96);
        assertEquals(TruthValues.TRUE,spec.evalThreeValuedSemantic("phi", 10, 0));
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
        spec.setSize(100);
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
    void loadSlowProperty() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        RobustnessFormula formula = spec.getFormula("phi_slow");
        assertNotNull(formula);
    }

    @Test
    @Disabled
    void loadSample() throws IOException{
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setM(1);
        ControlledSystem system = spec.getSystem();
        DataStateExpression f = spec.getPenalty("off");
        Perturbation perturbation = spec.getPerturbation("p_ItDistSens");
        double[] data = SystemState.sample(new DefaultRandomGenerator(), f, perturbation, system, 500, 100);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d> %f\n", i, data[i]);
        }
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
    void testApplyPerturbation() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        assertNotNull(spec.applyPerturbation("p_ItSlow_02", 0, 60, 400));
    }

    @Test
    void testEvalDistance() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        assertTrue(spec.evalDistanceExpression("exp_crash", "p_ItSlow_03", 0, 60)>0);
    }

    @Test
    void vehicleSlow02x10BoolCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        Boolean[] expected = new Boolean[10];
        Arrays.fill(expected,true);
        assertArrayEquals(expected, spec.evalBooleanSemantic("phi_slow_02", 60, 0,100,10));
    }

    @Test
    void vehicleSlow02x30BoolCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        Boolean[] expected = new Boolean[10];
        Arrays.fill(expected,true);
        assertArrayEquals(expected, spec.evalBooleanSemantic("phi_slow_02", 60, 0,300,30));
    }

    @Test
    void vehicleSlow02x50BoolCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        Boolean[] expected = new Boolean[10];
        Arrays.fill(expected,true);
        assertArrayEquals(expected, spec.evalBooleanSemantic("phi_slow_02", 60, 0,500,50));
    }

    @Test
    void vehicleComb04x10ThreeValCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.FALSE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_comb_04", 60, 0,100,10));
    }

    @Test
    void vehicleComb04x30ThreeValCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.FALSE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_comb_04", 60, 0,300,30));
    }


    @Test
    void vehicleComb04x50ThreeValCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setM(50);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[]{TruthValues.FALSE,TruthValues.FALSE,TruthValues.FALSE,TruthValues.FALSE,TruthValues.FALSE,TruthValues.FALSE,TruthValues.FALSE,TruthValues.TRUE,TruthValues.TRUE,TruthValues.TRUE};
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_comb_04", 60, 0,500,50));
    }

    @Test
    void vehicleSlowOffset03() throws IOException{
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setM(50);
        spec.setZ(1.96);
        assertEquals(TruthValues.UNKNOWN, spec.evalThreeValuedSemantic("phi_slow_03", 60,10));
    }

    @Test
    void vehicleCombOffset03() throws IOException{
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setM(50);
        spec.setZ(1.96);
        assertNotEquals(TruthValues.TRUE, spec.evalThreeValuedSemantic("phi_comb_03", 60,0));
    }

    @Test
    void vehicleCrashThreeValuedCheck005x10() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(VEHICLE)).openStream());
        spec.setM(40);
        spec.setZ(1.96);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.TRUE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_crash_speed", 60, 0,100,10));
    }

    @Test
    void loadSingleVehicleSpecification() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        assertNotNull(spec);
    }


    @Test
    void loadSingleVehicleSystem() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        ControlledSystem system = spec.getSystem();
        assertNotNull(system);
    }

    @Test
    void simulateSingleVehicleSystem() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        spec.setSize(100);
        assertNotNull(spec.getSamplesAt(50));
    }

    @Test
    void stepSingleVehicleSystem() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        ControlledSystem system = spec.getSystem();
        assertNotNull(system.sampleNext(new DefaultRandomGenerator()));
    }


    @Test
    void loadSingleVehicleProperty() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        RobustnessFormula formula = spec.getFormula("phi_slow_02");
        assertNotNull(formula);
    }

    @Test
    void loadSingleSample() throws IOException{
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        spec.setSize(100);
        ControlledSystem system = spec.getSystem();
        EvolutionSequence sequence = spec.getSequence();
        DataStateExpression f = spec.getPenalty("physical_dist");
        double[] data = SystemState.sample(new DefaultRandomGenerator(), f, system, 300, 100);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d> %f\n", i, data[i]);
        }
        DataStateExpression f2 = spec.getPenalty("sensed_speed");
        double[] data2 = SystemState.sample(new DefaultRandomGenerator(), f2, system, 300, 100);
        for (int i = 0; i < data2.length; i++) {
            System.out.printf("%d> %f\n", i, data2[i]);
        }
        DataStateExpression f3 = spec.getPenalty("physical_speed");
        double[] data3 = SystemState.sample(new DefaultRandomGenerator(), f3, system, 300, 100);
        for (int i = 0; i < data3.length; i++) {
            System.out.printf("%d> %f\n", i, data3[i]);
        }
        DataStateExpression f4 = spec.getPenalty("rho_token");
        double[] data4 = SystemState.sample(new DefaultRandomGenerator(), f4, system, 300, 100);
        for (int i = 0; i < data4.length; i++) {
            System.out.printf("%d> %f\n", i, data4[i]);
        }
    }


    @Test
    void testSinglePerturbation() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        spec.setSize(100);
        assertNotNull(spec.applyPerturbation("p_ItSlow", 0, 60, 400));
    }

    @Test
    void testSingleDistance() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        spec.setSize(100);
        ControlledSystem system = spec.getSystem();
        DataStateExpression f = spec.getPenalty("rho_crash");
        Perturbation perturbation = spec.getPerturbation("p_ItSlow_04");
        double[] data = SystemState.sample(new DefaultRandomGenerator(), f, perturbation, system, 400, 100);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d> %f\n", i, data[i]);
        }
        DataStateExpression f1 = spec.getPenalty("physical_dist");
        double[] data1 = SystemState.sample(new DefaultRandomGenerator(), f1, perturbation, system, 300, 100);
        for (int i = 0; i < data1.length; i++) {
            System.out.printf("%d> %f\n", i, data1[i]);
        }
        DataStateExpression f2 = spec.getPenalty("sensed_speed");
        double[] data2 = SystemState.sample(new DefaultRandomGenerator(), f2, perturbation, system, 300, 100);
        for (int i = 0; i < data2.length; i++) {
            System.out.printf("%d> %f\n", i, data2[i]);
        }
        DataStateExpression f3 = spec.getPenalty("rho_offset");
        double[] data3 = SystemState.sample(new DefaultRandomGenerator(), f3, perturbation, system, 300, 100);
        for (int i = 0; i < data3.length; i++) {
            System.out.printf("%d> %f\n", i, data3[i]);
        }
        assertTrue(spec.evalDistanceExpression("exp_crash", "p_ItSlow_04", 0, 50)>0);
    }

    @Test
    void singleVehicleSlow02BoolCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        spec.setSize(100);
        Boolean[] expected = new Boolean[35];
        Arrays.fill(expected,true);
        assertArrayEquals(expected, spec.evalBooleanSemantic("phi_slow_02", 10, 0,350,10));
    }

    @Test
    void singleVehicleSlow02x30Check() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        spec.setSize(100);
        TruthValues[] expected = new TruthValues[10];
        Arrays.fill(expected,TruthValues.TRUE);
        assertArrayEquals(expected, spec.evalThreeValuedSemantic("phi_slow_02", 10, 0,300,30));
    }

    @Test
    void singleVehicleSlow04BoolCheck() throws IOException {
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        spec.setSize(100);
        assertFalse(spec.evalBooleanSemantic("always_slow_04", 10, 0));
    }

    @Test
    void singleVehicleSlowOffset04() throws IOException{
        SpecificationLoader loader = new SpecificationLoader();
        SystemSpecification spec = loader.loadSpecification(Objects.requireNonNull(ClassLoader.getSystemClassLoader().getResource(SINGLE_VEHICLE)).openStream());
        spec.setSize(100);
        spec.setM(50);
        spec.setZ(1.96);
        assertEquals(TruthValues.FALSE, spec.evalThreeValuedSemantic("always_slow_04", 10,0));
    }


}