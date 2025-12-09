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

package it.unicam.quasylab.jspear.examples.vehicle;

import it.unicam.quasylab.jspear.ControlledSystem;
import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.Util;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ParallelController;
import it.unicam.quasylab.jspear.distance.AtomicDistanceExpressionLeq;
import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.distance.MaxIntervalDistanceExpression;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.perturbation.AfterPerturbation;
import it.unicam.quasylab.jspear.perturbation.AtomicPerturbation;
import it.unicam.quasylab.jspear.perturbation.IterativePerturbation;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class test_figure_3_right {

    public final static String[] VARIABLES =
            new String[]{"p_speed_V1", "s_speed_V1", "p_distance_V1", "s_distance_V1", "accel_V1", "timer_V1",
                    "warning_V1", "braking_distance_V1", "required_distance_V1", "safety_gap_V1",
                    "brake_light_V1",
                    "p_speed_V2", "s_speed_V2", "p_distance_V2", "s_distance_V2",
                    "p_distance_V1_V2", "s_distance_V1_V2", "accel_V2", "timer_V2",
                    "warning_V2", "braking_distance_V2", "required_distance_V2", "safety_gap_V2",
                    "safety_gap_V1_V2", "brake_light_V2", "crashed_V1", "crashed_V2"
            };
    public final static double ACCELERATION = 1.0;
    public final static double BRAKE = 2.0;
    public final static double NEUTRAL = 0.0;
    public final static int TIMER_INIT = 5;
    public final static int DANGER = 1;
    public final static int OK = 0;
    public final static double INIT_SPEED_V1 = 25.0;
    public final static double INIT_SPEED_V2 = 25.0;
    public final static double MAX_SPEED = 40.0;
    public final static double INIT_DISTANCE_OBS_V1 = 10000.0;
    public final static double INIT_DISTANCE_V1_V2 = 5000.0;
    private static final double SAFETY_DISTANCE = 200.0;

    private static final int H = 450;
    private static final double MAX_DISTANCE_OFFSET = 1.0;
    private static double ETA_CRASH_SPEED = 0.05;
    private static double ETA_CRASH_SPEED_BIS = 0.1;
    private static double ETA_CRASH_SPEED_TER = 0.15;
    private static final int p_speed_V1 = 0;//variableRegistry.getVariable("p_speed");
    private static final int s_speed_V1 = 1;//variableRegistry.getVariable("s_speed");
    private static final int p_distance_V1 = 2;// variableRegistry.getVariable("p_distance");
    private static final int s_distance_V1 = 3;// variableRegistry.getVariable("s_distance");
    private static final int accel_V1 = 4;//variableRegistry.getVariable("accel");
    private static final int timer_V1 = 5;//variableRegistry.getVariable("timer");
    private static final int warning_V1 = 6;//variableRegistry.getVariable("warning");
    private static final int braking_distance_V1 = 7;//variableRegistry.getVariable("braking_distance");
    private static final int required_distance_V1 = 8; //variableRegistry.getVariable("required_distance");
    private static final int safety_gap_V1 = 9;//variableRegistry.getVariable("safety_gap");
    private static final int brake_light_V1 = 10;

    private static final int p_speed_V2 = 11;//variableRegistry.getVariable("p_speed");
    private static final int s_speed_V2 = 12;//variableRegistry.getVariable("s_speed");
    private static final int p_distance_V2 = 13;// variableRegistry.getVariable("p_distance");
    private static final int s_distance_V2 = 14;// variableRegistry.getVariable("s_distance");
    private static final int p_distance_V1_V2 = 15;// variableRegistry.getVariable("p_distance");
    private static final int s_distance_V1_V2 = 16;
    private static final int accel_V2 = 17;//variableRegistry.getVariable("accel");
    private static final int timer_V2 = 18;//variableRegistry.getVariable("timer");
    private static final int warning_V2 = 19;//variableRegistry.getVariable("warning");
    private static final int braking_distance_V2 = 20;//variableRegistry.getVariable("braking_distance");
    private static final int required_distance_V2 = 21; //variableRegistry.getVariable("required_distance");
    private static final int safety_gap_V2 = 22;
    private static final int safety_gap_V1_V2 = 23;//variableRegistry.getVariable("safety_gap");
    private static final int brake_light_V2 = 24;
    private static final int crashed_V1 = 25;
    private static final int crashed_V2 = 26;

    private static final int NUMBER_OF_VARIABLES = 27;



    public static void main(String[] args) throws IOException {
        try {
            RandomGenerator rand = new DefaultRandomGenerator();
            Controller controller_V1 = getController_V1();
            Controller controller_V2 = getController_V2();
            DataState state = getInitialState();
            ControlledSystem system = new ControlledSystem(new ParallelController(controller_V1, controller_V2), (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, 1);

            DistanceExpression crash_speed = new AtomicDistanceExpressionLeq(test_figure_3_right::rho_crash_speed);

            RobustnessFormula Phi_crash_speed = new AlwaysRobustnessFormula(
                    new AtomicRobustnessFormula(getIteratedDistanceSensorsPerturbation(),
                            new MaxIntervalDistanceExpression(crash_speed, 10, 400),
                            RelationOperator.LESS_OR_EQUAL_THAN,
                            ETA_CRASH_SPEED),
                    0,
                    H);

            RobustnessFormula Phi_crash_speed_bis = new AlwaysRobustnessFormula(
                    new AtomicRobustnessFormula(getIteratedDistanceSensorsPerturbation(),
                            new MaxIntervalDistanceExpression(crash_speed, 10, 400),
                            RelationOperator.LESS_OR_EQUAL_THAN,
                            ETA_CRASH_SPEED_BIS),
                    0,
                    H);

            RobustnessFormula Phi_crash_speed_ter = new AlwaysRobustnessFormula(
                    new AtomicRobustnessFormula(getIteratedDistanceSensorsPerturbation(),
                            new MaxIntervalDistanceExpression(crash_speed, 10, 400),
                            RelationOperator.LESS_OR_EQUAL_THAN,
                            ETA_CRASH_SPEED_TER),
                    0,
                    H);


            // Tests on the three-valued evaluation of formulae

            double[][] val_crash_speed = new double[10][1];

            for(int i = 0; i<10; i++) {
                int step = i*50;
                TruthValues value = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(Phi_crash_speed).eval(60, step, sequence);
                System.out.println("Phi_crash_speed evaluation at step "+step+" with threshold "+ETA_CRASH_SPEED+": " + value);
                if (value == TruthValues.TRUE) {
                    val_crash_speed[i][0] = 1;
                } else {
                    if (value == TruthValues.UNKNOWN) {
                        val_crash_speed[i][0] = 0;
                    } else {
                        val_crash_speed[i][0] = -1;
                    }
                }
            }

            Util.writeToCSV("./phi_crash_speed_test_005x50.csv",val_crash_speed);

            for(int i = 0; i<10; i++) {
                int step = i*50;
                TruthValues value = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(Phi_crash_speed_bis).eval(60, step, sequence);
                System.out.println("Phi_crash_speed evaluation at step "+step+" with threshold "+ETA_CRASH_SPEED_BIS+": " + value);
                if (value == TruthValues.TRUE) {
                    val_crash_speed[i][0] = 1;
                } else {
                    if (value == TruthValues.UNKNOWN) {
                        val_crash_speed[i][0] = 0;
                    } else {
                        val_crash_speed[i][0] = -1;
                    }
                }
            }

            Util.writeToCSV("./phi_crash_speed_test_01x50.csv",val_crash_speed);

            for(int i = 0; i<10; i++) {
                int step = i*50;
                TruthValues value = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(Phi_crash_speed_ter).eval(60, step, sequence);
                System.out.println("Phi_crash_speed evaluation at step "+step+" with threshold "+ETA_CRASH_SPEED_TER+": " + value);
                if (value == TruthValues.TRUE) {
                    val_crash_speed[i][0] = 1;
                } else {
                    if (value == TruthValues.UNKNOWN) {
                        val_crash_speed[i][0] = 0;
                    } else {
                        val_crash_speed[i][0] = -1;
                    }
                }
            }

            Util.writeToCSV("./phi_crash_speed_test_015x50.csv",val_crash_speed);

        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }



    // PENALTY FUNCTIONS

    public static double rho_crash_speed(DataState state) {
        if (state.get(crashed_V2) == 0 && (state.get(p_distance_V2) <=0 || state.get(p_distance_V1_V2) <=0)){
            return state.get(p_speed_V2)/MAX_SPEED;
        }
        else{
            return 0.0;
        }
    }

    // CONTROLLER OF VEHICLE 1

    public static Controller getController_V1() {


        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Ctrl_V1",
                Controller.ifThenElse(
                        DataState.greaterThan(s_speed_V1, 0),
                        Controller.ifThenElse(
                                   DataState.greaterThan(safety_gap_V1, 0 ),
                                   Controller.doAction(
                                           (rg, ds) -> List.of(new DataStateUpdate(accel_V1, ACCELERATION), new DataStateUpdate(timer_V1, TIMER_INIT),
                                                   new DataStateUpdate(brake_light_V1, 0)),
                                           registry.reference("Accelerate_V1")
                                   ),
                                   Controller.doAction(
                                           (rg, ds) -> List.of( new DataStateUpdate(accel_V1, - BRAKE), new DataStateUpdate(timer_V1, TIMER_INIT),
                                                         new DataStateUpdate(brake_light_V1, 1)),
                                           registry.reference("Decelerate_V1"))
                        ),
                        Controller.doAction(
                                (rg,ds)-> List.of(new DataStateUpdate(accel_V1,NEUTRAL), new DataStateUpdate(timer_V1,TIMER_INIT)),
                                registry.reference("Stop_V1")
                        )
                )
        );

        registry.set("Accelerate_V1",
                Controller.ifThenElse(
                        DataState.greaterThan(timer_V1, 0),
                        Controller.doTick(registry.reference("Accelerate_V1")),
                        registry.reference("Ctrl_V1")
                )
        );

        registry.set("Decelerate_V1",
                Controller.ifThenElse(
                        DataState.greaterThan(timer_V1, 0),
                        Controller.doTick(registry.reference("Decelerate_V1")),
                        registry.reference("Ctrl_V1")
                )
        );

        registry.set("Stop_V1",
                Controller.ifThenElse(
                        DataState.greaterThan(timer_V1, 0),
                        Controller.doTick(registry.reference("Stop_V1")),
                        Controller.ifThenElse(
                                DataState.equalsTo(warning_V1,DANGER),
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(accel_V1, -BRAKE),
                                                new DataStateUpdate(timer_V1, TIMER_INIT),
                                                new DataStateUpdate(brake_light_V1,1)),
                                        registry.reference("Decelerate_V1")
                                ),
                                Controller.doAction(
                                        DataStateUpdate.set(timer_V1,TIMER_INIT),
                                        registry.reference("Stop_V1")
                                )
                        )
                )
        );

        registry.set("IDS_V1",
                Controller.ifThenElse(
                        DataState.lessOrEqualThan(p_distance_V1, 2*TIMER_INIT*SAFETY_DISTANCE).and(DataState.equalsTo(accel_V1, ACCELERATION).or(DataState.equalsTo(accel_V1, NEUTRAL).and(DataState.greaterThan(p_speed_V1,0.0)))),
                        Controller.doAction(DataStateUpdate.set(warning_V1, DANGER),registry.reference("IDS_V1")),
                        Controller.doAction(DataStateUpdate.set(warning_V1, OK),registry.reference("IDS_V1"))
                )
        );
        return new ParallelController(registry.reference("Ctrl_V1"), registry.reference("IDS_V1"));

    }

    //  CONTROLLER OF VEHICLE 2

    public static Controller getController_V2() {

        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Ctrl_V2",
                Controller.ifThenElse(
                        DataState.greaterThan(s_speed_V2, 0),
                        Controller.ifThenElse(
                                DataState.greaterThan(safety_gap_V1_V2, 0 ).and(DataState.equalsTo(brake_light_V1, 0 ).or(DataState.greaterOrEqualThan(s_distance_V1_V2, 300))).and(DataState.greaterThan(safety_gap_V2, 0 )),
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(accel_V2, ACCELERATION), new DataStateUpdate(timer_V2, TIMER_INIT),
                                                new DataStateUpdate(brake_light_V2, 0)),
                                        registry.reference("Accelerate_V2")),
                                Controller.doAction(
                                        (rg, ds) -> List.of( new DataStateUpdate(accel_V2, - BRAKE), new DataStateUpdate(timer_V2, TIMER_INIT),
                                                new DataStateUpdate(brake_light_V2, 1)),
                                        registry.reference("Decelerate_V2"))
                        ),
                        Controller.doAction(
                                (rg,ds)-> List.of(new DataStateUpdate(accel_V2,NEUTRAL), new DataStateUpdate(timer_V2,TIMER_INIT)),
                                registry.reference("Stop_V2")
                        )
                )
        );

        registry.set("Accelerate_V2",
                Controller.ifThenElse(
                        DataState.greaterThan(timer_V2, 0),
                        Controller.doTick(registry.reference("Accelerate_V2")),
                        registry.reference("Ctrl_V2")
                )
        );

        registry.set("Decelerate_V2",
                Controller.ifThenElse(
                        DataState.greaterThan(timer_V2, 0),
                        Controller.doTick(registry.reference("Decelerate_V2")),
                        registry.reference("Ctrl_V2")
                )
        );

        registry.set("Stop_V2",
                Controller.ifThenElse(
                        DataState.greaterThan(timer_V2, 0),
                        Controller.doTick(registry.reference("Stop_V2")),
                        Controller.ifThenElse(
                                DataState.equalsTo(warning_V2,DANGER),
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(accel_V2, -BRAKE),
                                                new DataStateUpdate(timer_V2, TIMER_INIT),
                                                new DataStateUpdate(brake_light_V2,1)),
                                        registry.reference("Decelerate_V2")
                                ),
                                Controller.doAction(
                                        DataStateUpdate.set(timer_V2,TIMER_INIT),
                                        registry.reference("Stop_V2")
                                )
                        )
                )
        );

        registry.set("IDS_V2",
                Controller.ifThenElse(
                        DataState.lessOrEqualThan(p_distance_V2, 2*TIMER_INIT*SAFETY_DISTANCE).and(DataState.equalsTo(accel_V2, ACCELERATION).or(DataState.equalsTo(accel_V2, NEUTRAL).and(DataState.greaterThan(p_speed_V2,0.0)))),
                        Controller.doAction(DataStateUpdate.set(warning_V2, DANGER),registry.reference("IDS_V2")),
                        Controller.doAction(DataStateUpdate.set(warning_V2, OK),registry.reference("IDS_V2"))
                )
        );
        return new ParallelController(registry.reference("Ctrl_V2"), registry.reference("IDS_V2"));
    }

    // ENVIRONMENT EVOLUTION

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double travel_V1 = state.get(accel_V1)/2 + state.get(p_speed_V1);
        double new_timer_V1 = state.get(timer_V1) - 1;
        double new_p_speed_V1 = Math.min(MAX_SPEED,Math.max(0,state.get(p_speed_V1) + state.get(accel_V1)));
        double new_p_distance_V1 = state.get(p_distance_V1) - travel_V1;
        double travel_V2 = state.get(accel_V2)/2 + state.get(p_speed_V2);
        double new_timer_V2 = state.get(timer_V2) - 1;
        double new_p_speed_V2 = Math.min(MAX_SPEED,Math.max(0,state.get(p_speed_V2) + state.get(accel_V2)));
        double new_p_distance_V1_V2 = state.get(p_distance_V1_V2) - travel_V2 + travel_V1;
        double new_p_distance_V2 = state.get(p_distance_V2) - travel_V2;
        updates.add(new DataStateUpdate(timer_V1, new_timer_V1));
        updates.add(new DataStateUpdate(p_speed_V1, new_p_speed_V1));
        updates.add(new DataStateUpdate(p_distance_V1, new_p_distance_V1));
        updates.add(new DataStateUpdate(timer_V2, new_timer_V2));
        updates.add(new DataStateUpdate(p_speed_V2, new_p_speed_V2));
        updates.add(new DataStateUpdate(p_distance_V2, new_p_distance_V2));
        updates.add(new DataStateUpdate(p_distance_V1_V2, new_p_distance_V1_V2));
        if(new_timer_V1 == 0) {
            double new_bd_V1 = (new_p_speed_V1 * new_p_speed_V1 + (ACCELERATION + BRAKE) * (ACCELERATION * TIMER_INIT * TIMER_INIT +
                    2 * new_p_speed_V1 * TIMER_INIT)) / (2 * BRAKE);
            double new_rd_V1 = new_bd_V1 + SAFETY_DISTANCE;
            double new_sg_V1 = new_p_distance_V1 - new_rd_V1;
            updates.add(new DataStateUpdate(s_speed_V1, new_p_speed_V1));
            updates.add(new DataStateUpdate(braking_distance_V1, new_bd_V1));
            updates.add(new DataStateUpdate(required_distance_V1, new_rd_V1));
            updates.add(new DataStateUpdate(safety_gap_V1, new_sg_V1));
            updates.add(new DataStateUpdate(s_distance_V1,new_p_distance_V1));
        }
        if(new_timer_V2 == 0) {
            double new_bd_V2 = (new_p_speed_V2 * new_p_speed_V2 + (ACCELERATION + BRAKE) * (ACCELERATION * TIMER_INIT * TIMER_INIT +
                    2 * new_p_speed_V2 * TIMER_INIT)) / (2 * BRAKE);
            double new_rd_V2 = new_bd_V2 + SAFETY_DISTANCE;
            double new_sg_V1_V2 = new_p_distance_V1_V2 - new_rd_V2;
            double new_sg_V2 = new_p_distance_V2 - new_rd_V2;
            updates.add(new DataStateUpdate(s_speed_V2, new_p_speed_V2));
            updates.add(new DataStateUpdate(braking_distance_V2, new_bd_V2));
            updates.add(new DataStateUpdate(required_distance_V2, new_rd_V2));
            updates.add(new DataStateUpdate(safety_gap_V1_V2, new_sg_V1_V2));
            updates.add(new DataStateUpdate(safety_gap_V2, new_sg_V2));
            updates.add(new DataStateUpdate(s_distance_V2, new_p_distance_V2));
            updates.add(new DataStateUpdate(s_distance_V1_V2, new_p_distance_V1_V2));
        }
        if(state.get(p_distance_V2) <=0 || state.get(p_distance_V1_V2) <=0){
            updates.add(new DataStateUpdate(crashed_V2, 1));
        }
        if(state.get(p_distance_V1) <=0){
            updates.add(new DataStateUpdate(crashed_V1, 1));
        }
        return updates;
    }



    // PERTURBATIONS

    private static  Perturbation getIteratedDistanceSensorsPerturbation() {
        return new AfterPerturbation(1, new IterativePerturbation(300, new AtomicPerturbation(TIMER_INIT - 1, test_figure_3_right::distanceSensorsPerturbation)));
    }

    private static DataState distanceSensorsPerturbation(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double travel_V1 = state.get(accel_V1)/2 + state.get(p_speed_V1);
        double travel_V2 = state.get(accel_V2)/2 + state.get(p_speed_V2);
        double new_p_distance_V1_V2 = state.get(p_distance_V1_V2) - travel_V2 + travel_V1;
        double new_p_distance_V2 = state.get(p_distance_V2) - travel_V2;
        double offset = rg.nextDouble() * MAX_DISTANCE_OFFSET;
        double offset_V1_V2 = new_p_distance_V1_V2 * offset;
        double offset_V2 = new_p_distance_V2 * offset;
        double noisy_distance_V1_V2 = new_p_distance_V1_V2  + offset_V1_V2;
        double noisy_distance_V2 = new_p_distance_V2  + offset_V2;
        double new_p_speed_V2 = Math.min(MAX_SPEED,Math.max(0,state.get(p_speed_V2) + state.get(accel_V2)));
        double new_bd_V2 = (new_p_speed_V2 * new_p_speed_V2 + (ACCELERATION + BRAKE) * (ACCELERATION * TIMER_INIT * TIMER_INIT +
                2 * new_p_speed_V2 * TIMER_INIT)) / (2 * BRAKE);
        double new_rd_V2 = new_bd_V2 + SAFETY_DISTANCE;
        double new_sg_V1_V2 = noisy_distance_V1_V2 - new_rd_V2;
        double new_sg_V2 = noisy_distance_V2 - new_rd_V2;
        updates.add(new DataStateUpdate(safety_gap_V1_V2, new_sg_V1_V2));
        updates.add(new DataStateUpdate(safety_gap_V2, new_sg_V2));
        updates.add(new DataStateUpdate(s_distance_V1_V2,noisy_distance_V1_V2));
        updates.add(new DataStateUpdate(s_distance_V2,noisy_distance_V2));
        return state.apply(updates);
    }


   // INITIALISATION OF DATA STATE

    public static DataState getInitialState( ) {
        Map<Integer, Double> values = new HashMap<>();
        // INITIAL DATA FOR V1
        values.put(crashed_V1, (double) 0);
        values.put(brake_light_V1, (double) 0);
        values.put(timer_V1, (double) 0);
        values.put(p_speed_V1, INIT_SPEED_V1);
        values.put(s_speed_V1, INIT_SPEED_V1);
        values.put(p_distance_V1, INIT_DISTANCE_OBS_V1);
        values.put(s_distance_V1, INIT_DISTANCE_OBS_V1);
        values.put(accel_V1, NEUTRAL);
        values.put(warning_V1, (double) OK);
        double init_bd_V1 = (INIT_SPEED_V1 * INIT_SPEED_V1 + (ACCELERATION + BRAKE) * (ACCELERATION * TIMER_INIT * TIMER_INIT +
                2 * INIT_SPEED_V1 * TIMER_INIT))/(2 * BRAKE);
        double init_rd_V1 = init_bd_V1 + SAFETY_DISTANCE;
        double init_sg_V1 = INIT_DISTANCE_OBS_V1 - init_rd_V1;
        values.put(braking_distance_V1, init_bd_V1);
        values.put(required_distance_V1, init_rd_V1);
        values.put(safety_gap_V1, init_sg_V1);
        // INITIAL DATA FOR V2
        values.put(crashed_V2, (double) 0);
        values.put(timer_V2, (double) 0);
        values.put(brake_light_V2, (double) 0);
        values.put(p_speed_V2, INIT_SPEED_V2);
        values.put(s_speed_V2, INIT_SPEED_V2);
        values.put(p_distance_V2, INIT_DISTANCE_V1_V2 + INIT_DISTANCE_OBS_V1);
        values.put(s_distance_V2, INIT_DISTANCE_V1_V2 + INIT_DISTANCE_OBS_V1);
        values.put(p_distance_V1_V2, INIT_DISTANCE_V1_V2 );
        values.put(s_distance_V1_V2, INIT_DISTANCE_V1_V2 );
        values.put(accel_V2, NEUTRAL);
        values.put(warning_V2, (double) OK);
        double init_bd_V2 = (INIT_SPEED_V2 * INIT_SPEED_V2 + (ACCELERATION + BRAKE) * (ACCELERATION * TIMER_INIT * TIMER_INIT +
                2 * INIT_SPEED_V2 * TIMER_INIT))/(2 * BRAKE);
        double init_rd_V2 = init_bd_V2 + SAFETY_DISTANCE;
        double init_sg_V1_V2 = INIT_DISTANCE_V1_V2 - init_rd_V2;
        double init_sg_V2 = INIT_DISTANCE_V1_V2 + INIT_DISTANCE_OBS_V1- init_rd_V2;
        values.put(braking_distance_V2, init_bd_V2);
        values.put(required_distance_V2, init_rd_V2);
        values.put(safety_gap_V1_V2, init_sg_V1_V2);
        values.put(safety_gap_V2, init_sg_V2);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }

}
