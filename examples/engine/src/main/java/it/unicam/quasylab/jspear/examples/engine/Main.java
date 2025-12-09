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

package it.unicam.quasylab.jspear.examples.engine;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ParallelController;
import it.unicam.quasylab.jspear.distance.*;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.perturbation.*;
import it.unicam.quasylab.jspear.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.*;
import java.util.stream.DoubleStream;

public class Main {

    public final static String[] VARIABLES =
            new String[] { "P1", "P2", "P3", "P4", "P5", "P6", "stress",  "temp", "cool", "speed", "ch_temp", "ch_wrn", "ch_speed", "ch_out", "ch_in"};
    public final static double ON = 0;
    public final static double OFF = 1;
    public final static double SLOW = 2;
    public final static double HALF = 3;
    public final static double FULL = 4;
    public final static double OK = 5;
    public final static double HOT = 6;
    public final static double LOW = 7;
    private static final double MIN_TEMP = 0;
    private static final double MAX_TEMP = 150;
    private static final double STRESS_INCR = 0.1;
    //private static final VariableAllocation variableRegistry = VariableAllocation.create(VARIABLES);
    private static final int p1 = 0;//variableRegistry.getVariable("P1");
    private static final int p2 = 1;//variableRegistry.getVariable("P2");
    private static final int p3 = 2;//variableRegistry.getVariable("P3");
    private static final int p4 = 3;//variableRegistry.getVariable("P4");
    private static final int p5 = 4;//variableRegistry.getVariable("P5");
    private static final int p6 = 5;//variableRegistry.getVariable("P6");
    private static final int stress = 6;//variableRegistry.getVariable("stress");
    public static final int temp = 7;//variableRegistry.getVariable("temp");
    private static final int ch_temp = 8;//variableRegistry.getVariable("ch_temp");
    public static final int cool = 9;//variableRegistry.getVariable("cool");
    private static final int ch_speed = 10;//variableRegistry.getVariable("speed");
    private static final int ch_wrn = 11;//variableRegistry.getVariable("ch_wrn");
    private static final int ch_in = 12;//variableRegistry.getVariable("ch_in");
    private static final int ch_out = 13;
    private static final int fn = 14;
    private static final int fp = 15;
    private static final int counter = 16;//

    private static final int NUMBER_OF_VARIABLES = 17;//
    private static final double INITIAL_TEMP_VALUE = 95.0;
    private static final int N = 100;
    private static final int TAU = 100;
    private static final int TAU2 = 250;
    private static final int TAU3 = 300;
    private static final int K = TAU+N+10;
    private static final int H = 1000;
    private static final double TEMP_OFFSET_1 = -1.0;
    private static final double TEMP_OFFSET_15 = -1.5;
    private static final double TEMP_OFFSET_2 = -2.0;
    private static final double COOL_OFFSET = 1.8;
    private static final double ETA1 = 0.0;
    private static final double ETA2 = 0.02;
    private static final double ETA3 = 0.05;
    private static final double ETA4 = 0.3;

    private static final double ETA003 = 0.03;
    private static final double ETA004 = 0.04;
    private static final double ETA005 = 0.05;
    private static final double ETA006 = 0.06;


    public static void main(String[] args) throws IOException {
        try {
            RandomGenerator rand = new DefaultRandomGenerator();

            Controller controller = getController();
            DataState state = getInitialState(INITIAL_TEMP_VALUE);
            ControlledSystem system = new ControlledSystem(controller, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, 100);

            DistanceExpression temp_atomic = new AtomicDistanceExpressionLeq(ds -> Math.abs((ds.get(temp)-ds.get(ch_temp))/Math.abs(MAX_TEMP-MIN_TEMP)));
            DistanceExpression temp_eventually = new MinIntervalDistanceExpression(
                    temp_atomic,
                    TAU,
                    TAU + N
            );
            DistanceExpression temp_always = new MaxIntervalDistanceExpression(
                    temp_atomic,
                    TAU,
                    TAU + N
            );

            DistanceExpression warning_atomic = new AtomicDistanceExpressionLeq(ds -> (ds.get(ch_wrn)==HOT?1.0:0.0));
            DistanceExpression warning_always = new MaxIntervalDistanceExpression(
                    warning_atomic,
                    TAU,
                    TAU + N
            );

            DistanceExpression stress_atomic = new AtomicDistanceExpressionLeq(ds -> ds.get(stress));
            DistanceExpression stress_always = new MaxIntervalDistanceExpression(
                    stress_atomic,
                    TAU,
                    TAU + N
            );

            RobustnessFormula Phi1_1_1_1 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_1),
                    temp_eventually,
                    RelationOperator.GREATER_OR_EQUAL_THAN,
                    ETA1
            );
            RobustnessFormula Phi1_1_2_1 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_1),
                    temp_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA2
            );
            RobustnessFormula Phi1_2_1_1 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_1),
                    warning_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA3
            );
            RobustnessFormula Phi1_2_2_1 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_1),
                    stress_always,
                    RelationOperator.GREATER_THAN,
                    ETA4
            );
            RobustnessFormula Phi1_1 = new EventuallyRobustnessFormula(
                    new ImplicationRobustnessFormula(
                            new ConjunctionRobustnessFormula(Phi1_1_1_1, Phi1_1_2_1),
                            new ConjunctionRobustnessFormula(Phi1_2_1_1, Phi1_2_2_1)
                    ),
                    0,
                    H
            );

            RobustnessFormula Phi1_1_1_15 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_15),
                    temp_eventually,
                    RelationOperator.GREATER_OR_EQUAL_THAN,
                    ETA1
            );
            RobustnessFormula Phi1_1_2_15 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_15),
                    temp_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA2
            );
            RobustnessFormula Phi1_2_1_15 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_15),
                    warning_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA3
            );
            RobustnessFormula Phi1_2_2_15 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_15),
                    stress_always,
                    RelationOperator.GREATER_THAN,
                    ETA4
            );
            RobustnessFormula Phi1_15 = new EventuallyRobustnessFormula(
                    new ImplicationRobustnessFormula(
                            new ConjunctionRobustnessFormula(Phi1_1_1_15, Phi1_1_2_15),
                            new ConjunctionRobustnessFormula(Phi1_2_1_15, Phi1_2_2_15)
                    ),
                    0,
                    H
            );

            RobustnessFormula Phi1_1_1_2 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_2),
                    temp_eventually,
                    RelationOperator.GREATER_OR_EQUAL_THAN,
                    ETA1
            );
            RobustnessFormula Phi1_1_2_2 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_2),
                    temp_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA2
            );
            RobustnessFormula Phi1_2_1_2 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_2),
                    warning_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA3
            );
            RobustnessFormula Phi1_2_2_2 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_2),
                    stress_always,
                    RelationOperator.GREATER_THAN,
                    ETA4
            );
            RobustnessFormula Phi1_2 = new EventuallyRobustnessFormula(
                    new ImplicationRobustnessFormula(
                            new ConjunctionRobustnessFormula(Phi1_1_1_2, Phi1_1_2_2),
                            new ConjunctionRobustnessFormula(Phi1_2_1_2, Phi1_2_2_2)
                    ),
                    0,
                    H
            );

            DistanceExpression until_distance = new UntilDistanceExpression(
                    new ThresholdDistanceExpression(stress_atomic,RelationOperator.LESS_THAN,0.3),
                    0,
                    K,
                    new ThresholdDistanceExpression(warning_atomic,RelationOperator.GREATER_THAN, 0.1)
            );

            RobustnessFormula Phi2 = new AtomicRobustnessFormula(perturbation_cool(COOL_OFFSET),
                    until_distance,
                    RelationOperator.LESS_THAN,
                    1.0
            );

            DistanceExpression false_negative = new AtomicDistanceExpressionLeq(
                    ds -> ds.get(fn)
            );

            RobustnessFormula Phi3 = new UntilRobustnessFormula(
                    Phi2,
                    0,
                    K,
                    new AtomicRobustnessFormula(perturbation_cool(COOL_OFFSET),
                            false_negative,
                            RelationOperator.LESS_OR_EQUAL_THAN,
                            ETA3)
            );

            DistanceExpression temp_expr = new AtomicDistanceExpressionLeq(ds -> (ds.get(temp)/Math.abs(MAX_TEMP-MIN_TEMP)));

            EvolutionSequence sequence_pert_temp_1 = sequence.apply(perturbation_temp(TEMP_OFFSET_1),0, 100);
            EvolutionSequence sequence_pert_temp_15 = sequence.apply(perturbation_temp(TEMP_OFFSET_15),0, 100);
            EvolutionSequence sequence_pert_temp_2 = sequence.apply(perturbation_temp(TEMP_OFFSET_2),0, 100);
            EvolutionSequence sequence_pert_cool = sequence.apply(perturbation_cool(COOL_OFFSET),TAU2, 100);

            System.out.println("Starting tests on temperature");
            Util.writeToCSV("./testTemperature.csv", Util.evalDistanceExpression(sequence, sequence_pert_temp_1, 90, 300, temp_expr));
            Util.writeToCSV("./testTemperature_l1.csv", Util.evalDistanceExpression(sequence, sequence_pert_temp_15, 90, 300, temp_expr));
            Util.writeToCSV("./testTemperature_l2.csv", Util.evalDistanceExpression(sequence, sequence_pert_temp_2, 90, 300, temp_expr));
            System.out.println("Tests on temperature completed");

            System.out.println("Starting tests on stress");
            Util.writeToCSV("./testStress.csv", Util.evalDistanceExpression(sequence, sequence_pert_temp_1, 90, 220, stress_atomic));
            Util.writeToCSV("./testStress_l1.csv", Util.evalDistanceExpression(sequence, sequence_pert_temp_15, 90, 220, stress_atomic));
            Util.writeToCSV("./testStress_l2.csv", Util.evalDistanceExpression(sequence, sequence_pert_temp_2, 90, 220, stress_atomic));
            System.out.println("Tests on stress completed");

            int l = 50;

            System.out.println("Starting tests on warning/stress");

            double[][] warn = new double[l][1];
            double[][] st = new double[l][1];
            for(int i=0; i<l; i++){
                EvolutionSequence sequence3 = sequence.apply(perturbation_temp(TEMP_OFFSET_15),i, 100);
                warn[i][0] = warning_always.compute(i,sequence,sequence3);
                st[i][0] = stress_always.compute(i,sequence,sequence3);
            }
            Util.writeToCSV("./testIntervalWarn.csv",warn);
            Util.writeToCSV("./testIntervalSt.csv",st);

            System.out.println("Tests on warning/stress ended");

            // BOOTSTRAP
            double[][] CI_left_50 = new double[l][1];
            double[][] CI_right_50 = new double[l][1];
            double[][] CI_left_100 = new double[l][1];
            double[][] CI_right_100 = new double[l][1];

            System.out.println("Starting tests on 50 bootstrap");

            for(int i=0; i<l; i++) {
                double[] testBoost_50 = warning_always.evalCI(rand,i,sequence,sequence_pert_temp_15,50,1.96);
                CI_left_50[i][0] = testBoost_50[1];
                CI_right_50[i][0] = testBoost_50[2];
            }
            System.out.println("Tests on 50 bootstrap ended");

            System.out.println("Starting tests on 100 bootstrap");
            for(int i=0; i<l; i++) {
                double[] testBoost_100 = warning_always.evalCI(rand,i,sequence,sequence_pert_temp_15,100,1.96);
                CI_left_100[i][0] = testBoost_100[1];
                CI_right_100[i][0] = testBoost_100[2];
            }
            System.out.println("Tests on 100 bootstrap ended");

            Util.writeToCSV("./testBootstrapL_50.csv",CI_left_50);
            Util.writeToCSV("./testBootstrapR_50.csv",CI_right_50);
            Util.writeToCSV("./testBootstrapL_100.csv",CI_left_100);
            Util.writeToCSV("./testBootstrapR_100.csv",CI_right_100);

            // FORMULAE
            RobustnessFormula Phi_003 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_15),
                    warning_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA003
            );
            RobustnessFormula Phi_004 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_15),
                    warning_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA004
            );
            RobustnessFormula Phi_005 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_15),
                    warning_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA005
            );
            RobustnessFormula Phi_006 = new AtomicRobustnessFormula(perturbation_temp(TEMP_OFFSET_15),
                    warning_always,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA006
            );

            double[][] val_003 = new double[50][1];
            double[][] val_004 = new double[50][1];
            double[][] val_005 = new double[50][1];
            double[][] val_006 = new double[50][1];

            System.out.println("Starting tests on 3-valued");
            for(int i = 0; i<50; i++) {
                TruthValues value1 = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(Phi_003).eval(60, i, sequence);
                System.out.println("Phi_003 evaluation at step "+i+": " + value1);
                if (value1 == TruthValues.TRUE) {
                    val_003[i][0] = 1;
                } else {
                    if (value1 == TruthValues.UNKNOWN) {
                        val_003[i][0] = 0;
                    } else {
                        val_003[i][0] = -1;
                    }
                }
                TruthValues value2 = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(Phi_004).eval(60, i, sequence);
                System.out.println("Phi_004 evaluation at step "+i+": " + value2);
                if (value2 == TruthValues.TRUE) {
                    val_004[i][0] = 1;
                } else {
                    if (value2 == TruthValues.UNKNOWN) {
                        val_004[i][0] = 0;
                    } else {
                        val_004[i][0] = -1;
                    }
                }
                TruthValues value3 = new ThreeValuedSemanticsVisitor(rand, 50, 1.96).eval(Phi_005).eval(60, i, sequence);
                System.out.println("Phi_005 evaluation at step " + i + ": " + value3);
                if (value3 == TruthValues.TRUE) {
                    val_005[i][0] = 1;
                } else {
                    if (value3 == TruthValues.UNKNOWN) {
                        val_005[i][0] = 0;
                    } else {
                        val_005[i][0] = -1;
                    }
                }
                TruthValues value4 = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(Phi_006).eval(60, i, sequence);
                System.out.println("Phi_006 evaluation at step "+i+": " + value4);
                if (value4 == TruthValues.TRUE) {
                    val_006[i][0] = 1;
                } else {
                    if (value4 == TruthValues.UNKNOWN) {
                        val_006[i][0] = 0;
                    } else {
                        val_006[i][0] = -1;
                    }
                }
            }
            System.out.println("Tests on 3-valued ended");

            Util.writeToCSV("./new_testThreeValue1.csv",val_003);
            Util.writeToCSV("./new_testThreeValue2.csv",val_004);
            Util.writeToCSV("./additional_testThreeValue.csv",val_005);
            Util.writeToCSV("./new_testThreeValue3.csv",val_006);

        } catch (RuntimeException e) {
            e.printStackTrace();
        }
   }

    public static ControllerRegistry getControllerRegistry() {
        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Ctrl",
                Controller.ifThenElse(
                        DataState.greaterOrEqualThan(ch_temp, 99.8),
                        Controller.doAction(DataStateUpdate.set(cool, ON), registry.reference("Cooling")),
                        registry.reference("Check")
                ));
        registry.set("Cooling",Controller.doTick(4, registry.reference("Check")));
        registry.set("Check",
                Controller.ifThenElse(
                        DataState.equalsTo(ch_speed, SLOW),
                        Controller.doAction(
                                (rg, ds) -> List.of (new DataStateUpdate(ch_speed, SLOW), new DataStateUpdate(cool, OFF)),registry.reference("Ctrl")),
                        Controller.doAction(
                                (rg, ds) -> List.of( new DataStateUpdate(ch_speed, ds.get(ch_in)), new DataStateUpdate(cool, OFF)),registry.reference("Ctrl"))
                )
        );
        registry.set("IDS",
                Controller.ifThenElse(
                        DataState.greaterThan(temp, 101.0).and(DataState.equalsTo(cool, OFF)),
                        Controller.doAction(
                                (rd, ds) -> List.of(new DataStateUpdate(ch_wrn, HOT), new DataStateUpdate(ch_speed, LOW), new DataStateUpdate(ch_out, FULL)),registry.reference("IDS")),
                        Controller.doAction(
                                (rg, ds) -> List.of(new DataStateUpdate(ch_wrn, OK), new DataStateUpdate(ch_speed, HALF), new DataStateUpdate(ch_out, HALF)),registry.reference("IDS"))
                )
        );
        return registry;
    }

    public static Controller getController() {
        ControllerRegistry registry = getControllerRegistry();
        return new ParallelController(registry.reference("Ctrl"), registry.reference("IDS"));
    }

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        double vP1 = state.get(p1);
        double vP2 = state.get(p2);
        double vP3 = state.get(p3);
        double vP4 = state.get(p4);
        double vP5 = state.get(p5);
        double vP6 = state.get(p6);
        double vTemp = state.get(temp);
        double vStress = state.get(stress);
        double vCool = state.get(cool);
        double vSpeed = state.get(ch_speed);
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(p1, vTemp));
        updates.add(new DataStateUpdate(p2, vP1));
        updates.add(new DataStateUpdate(p3, vP2));
        updates.add(new DataStateUpdate(p4, vP3));
        updates.add(new DataStateUpdate(p5, vP4));
        updates.add(new DataStateUpdate(p6, vP5));
        if (isStressing(vP1, vP2, vP3, vP4, vP5, vP6)) {
            updates.add(new DataStateUpdate(stress,Math.max(0.0,Math.min(1,vStress+STRESS_INCR))));
        }
        double newTemp = nextTempValue(vTemp, getTemperatureVariation(rg, vCool, vSpeed));
        updates.add(new DataStateUpdate(temp, newTemp));
        updates.add(new DataStateUpdate(ch_temp, newTemp));
        double new_fn = (state.get(counter)*state.get(fn) + Math.max(0.0,state.get(stress) - state.get(ch_wrn)))/(state.get(counter)+1);
        double new_fp = (state.get(counter)*state.get(fp) + Math.max(0.0,state.get(ch_wrn) - state.get(stress)))/(state.get(counter)+1);
        updates.add(new DataStateUpdate(fn, new_fn));
        updates.add(new DataStateUpdate(fp, new_fp));
        updates.add(new DataStateUpdate(counter, state.get(counter)+1));
        return updates;
    }

    private static Perturbation perturbation_temp(double offset) {
        //return new IterativePerturbation(N, new AtomicPerturbation(0, Main::perturbationFunction));
        return new AfterPerturbation(100,
                new IterativePerturbation(N, new AtomicPerturbation(0, (rg,ds)->f_temp(rg,ds,offset))));
    }

    private static DataState f_temp(RandomGenerator rg, DataState state, double offset) {
        double vTemp = state.get(temp);
        return state.apply(List.of(new DataStateUpdate(ch_temp, vTemp+ rg.nextDouble()*offset)));
    }

    private static Perturbation perturbation_cool(double offset) {
        //return new IterativePerturbation(N, new AtomicPerturbation(0, Main::perturbationFunction));
        return new IterativePerturbation(N, new AtomicPerturbation(0, (rg,ds)->f_cool(rg,ds,offset)));
    }

    private static DataState f_cool(RandomGenerator rg, DataState state, double offset) {
        double vTemp = state.get(temp);
        if (vTemp >= 99.8 - offset) {
            return state;
        } else {
            return state.apply(List.of(new DataStateUpdate(cool, OFF)));
        }
    }

    private static double nextTempValue(double vTemp, double v) {
        return Math.max(MIN_TEMP, Math.min(MAX_TEMP, vTemp+v));
    }

    private static double getTemperatureVariation(RandomGenerator rg, double vCool, double vSpeed) {
        if (vCool == ON) {
            return -1.2 + rg.nextDouble() * 0.4;
        }
        if (vSpeed == SLOW) {
            return 0.1 + rg.nextDouble() * 0.2;
        }
        if (vSpeed == HALF) {
            return 0.3 + rg.nextDouble() * 0.4;
        }
        return 0.7 + rg.nextDouble() * 0.5;
    }

    public static boolean isStressing(double vP1, double vP2, double vP3,double vP4, double vP5, double vP6) {
        return DoubleStream.of(vP1, vP2, vP3, vP4, vP5, vP6).filter(d -> d>=100).count()>3;
    }

    public static DataState getInitialState(double vTemp) {
        Map<Integer, Double> values = new HashMap<>();
        values.put(temp, vTemp);
        values.put(cool, OFF);
        values.put(ch_speed, HALF);
        values.put(ch_in, HALF);
        values.put(ch_temp, vTemp);
        values.put(p1, vTemp);
        values.put(p2, vTemp);
        values.put(p3, vTemp);
        values.put(p4, vTemp);
        values.put(p5, vTemp);
        values.put(p6, vTemp);
        values.put(fn, 0.0);
        values.put(fp, 0.0);
        values.put(counter, 0.0);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, 0.0));
    }
}
