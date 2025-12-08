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

package it.unicam.quasylab.jspear.examples.tollbooth;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.perturbation.*;
import it.unicam.quasylab.jspear.distl.*;
import it.unicam.quasylab.jspear.penalty.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.*;

public class Main {

    public final static String[] VARIABLES =
            new String[]{"p_speed", "s_speed", "p_distance", "accel", "timer_V", "braking_distance", "gap"
            };

    public final static double ACCELERATION = 0.25;
    public final static double BRAKE = 2.0;
    public final static double NEUTRAL = 0.0;
    public final static int TIMER = 1;
    public final static double INIT_SPEED = 25.0;
    public final static double MAX_SPEED = 40.0;
    public final static double INIT_DISTANCE = 10000.0;
    private static final int H = 350;

    private static final int p_speed = 0;//variableRegistry.getVariable("p_speed");
    private static final int s_speed = 1;//variableRegistry.getVariable("s_speed");
    private static final int p_distance = 2;// variableRegistry.getVariable("p_distance");
    private static final int accel = 3;//variableRegistry.getVariable("accel");
    private static final int timer_V = 4;//variableRegistry.getVariable("timer");
    private static final int braking_distance = 5;//variableRegistry.getVariable("braking_distance");
    private static final int gap = 6;//variableRegistry.getVariable("safety_gap");

    private static final int NUMBER_OF_VARIABLES = 7;



    public static void main(String[] args) throws IOException {
        try {
            Controller vehicle = getController();
            DataState state = getInitialState();
            ControlledSystem system = new ControlledSystem(vehicle, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);
            EvolutionSequence sequence = new EvolutionSequence(new DefaultRandomGenerator(), rg -> system, 100);

            DataStateFunction mu_pos = (rg, ds) -> ds.apply(getTargetPosDistribution(rg, ds));

            Penalty till_100 = new IterativePenalty(100,new AtomicPenalty(0,Main::rho_100));

            Penalty till_200 = new IterativePenalty(100,new AtomicPenalty(0,Main::rho_200));

            Penalty till_275 = new IterativePenalty(75,new AtomicPenalty(0,Main::rho_275));

            Penalty till_350 = new IterativePenalty(76,new AtomicPenalty(0,Main::rho_350));

            Penalty rho_pos = new SequentialPenalty(till_100, new SequentialPenalty(till_200, new SequentialPenalty(till_275, till_350)));

            DisTLFormula phi_pos = new TargetDisTLFormula(mu_pos, ds -> ds.get(p_distance), 0.3);

            DisTLFormula phi_pos2 = new TargetDisTLFormula(mu_pos, rho_pos, 0.3);

            DisTLFormula phi_pos_01 = new TargetDisTLFormula(mu_pos, rho_pos, 0.1);

            DisTLFormula phi_pos_03 = new TargetDisTLFormula(mu_pos, rho_pos, 0.3);

            DisTLFormula phi_pos_05 = new TargetDisTLFormula(mu_pos, rho_pos, 0.5);

            DataStateFunction mu_sp = (rg,ds) -> ds.apply(getTargetSpDistribution(rg, ds));

            DisTLFormula phi_sp = new TargetDisTLFormula(mu_sp, ds -> ds.get(p_speed)/MAX_SPEED, 0.0);

            DisTLFormula phi_1 = new AlwaysDisTLFormula(phi_pos,290,H);

            DisTLFormula phi_2 = new EventuallyDisTLFormula(new ConjunctionDisTLFormula(phi_sp,phi_pos2),0,H);

            //DisTLFormula phi_3 = new AlwaysDisTLFormula(phi_pos2,250,H);

            //DisTLFormula phi_4 = new EventuallyDisTLFormula(phi_pos2,0,H);

            double value_1 = new DoubleSemanticsVisitor().eval(phi_1).eval(10, 0, sequence);

            System.out.println("Robustness of the vehicle, in 0, wrt phi_1: "+value_1);

            double value_2 = new DoubleSemanticsVisitor().eval(phi_2).eval(10, 0, sequence);

            System.out.println("Robustness of the vehicle, in 0, wrt phi_2: "+value_2);

            //double value_3 = new DoubleSemanticsVisitor().eval(phi_3).eval(10, 0, sequence);

            //System.out.println("Robustness of the vehicle, in 0, wrt phi_3: "+value_3);

            //double value_4 = new DoubleSemanticsVisitor().eval(phi_4).eval(10, 0, sequence);

            //System.out.println("Robustness of the vehicle, in 0, wrt phi_4: "+value_4);

            double[][] value_01 = new double[31][1];
            double[][] value_03 = new double[31][1];
            double[][] value_05 = new double[31][1];

            for(int i = 0; i<31; i++) {
                DisTLFormula phi_01 = new AlwaysDisTLFormula(phi_pos_01,i+250,H);
                value_01[i][0] = new DoubleSemanticsVisitor().eval(phi_01).eval(10, 0, sequence);
                DisTLFormula phi_03 = new AlwaysDisTLFormula(phi_pos_03,i+250,H);
                value_03[i][0] = new DoubleSemanticsVisitor().eval(phi_03).eval(10, 0, sequence);
                DisTLFormula phi_05 = new AlwaysDisTLFormula(phi_pos_05,i+250,H);
                value_05[i][0] = new DoubleSemanticsVisitor().eval(phi_05).eval(10, 0, sequence);
            }

            Util.writeToCSV("./always_evaluation_01.csv",value_01);
            Util.writeToCSV("./always_evaluation_03.csv",value_03);
            Util.writeToCSV("./always_evaluation_05.csv",value_05);

            double[][] value = new double[351][1];
            for(int i = 0; i<351; i++) {
                value[i][0] = new DoubleSemanticsVisitor().eval(phi_2).eval(10, i, sequence);
            }

            Util.writeToCSV("./eventually_evaluation.csv",value);


            /*
            ArrayList<String> L = new ArrayList<>();

            L.add("speed");

            L.add("Distance");

            //L.add("gap");

            //L.add("accel");


            ArrayList<DataStateExpression> F = new ArrayList<>();

            F.add(ds->ds.get(s_speed));

            F.add(ds->ds.get(p_distance));

            //F.add(ds->ds.get(gap));

            //F.add(ds->ds.get(accel));

            printLData(new DefaultRandomGenerator(), L, F, system, 350, 100);

            printLData_min(new DefaultRandomGenerator(), L, F, system, 350, 100);

            printLData_max(new DefaultRandomGenerator(), L, F, system, 350, 100);

             */

        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }

    private static void printData(RandomGenerator rg, String label, DataStateExpression f, SystemState s, int steps, int size) {
        System.out.println(label);
        double[] data = SystemState.sample(rg, f, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d> %f\n", i, data[i]);
        }
    }

    private static void printData(RandomGenerator rg, String label, DataStateExpression f, Perturbation p, SystemState s, int steps, int size) {
        System.out.println(label);
        double[] data = SystemState.sample(rg, f, p, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d> %f\n", i, data[i]);
        }
    }

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, Perturbation p, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, p, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }

    private static void printLData_min(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample_min(rg, F, new NonePerturbation(), s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }

    private static void printLData_max(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample_max(rg, F, new NonePerturbation(), s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }

    // PENALTIES

    public static double rho_100(DataState state){
        return state.get(p_distance)/INIT_DISTANCE;
    }

    public static double rho_200(DataState state){
        return state.get(p_distance)/7000;
    }

    public static double rho_275(DataState state){
        return state.get(p_distance)/2500;
    }

    public static double rho_350(DataState state){
        return state.get(p_distance)/10;
    }

    // DISTRIBUTIONS

    public static List<DataStateUpdate> getTargetPosDistribution(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        Random r = new Random();
        updates.add(new DataStateUpdate(p_distance, r.nextGaussian()*0.2));
        return updates;
    }

    public static List<DataStateUpdate> getTargetSpDistribution(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(p_speed, 0.0));
        return updates;
    }

    // CONTROLLER OF VEHICLE

    public static Controller getController() {

        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Ctrl",
                Controller.ifThenElse(
                        DataState.greaterThan(s_speed, 0),
                        Controller.ifThenElse(
                                DataState.greaterThan(gap, 0 ),
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(accel, ACCELERATION),
                                                new DataStateUpdate(timer_V, TIMER)),
                                        registry.reference("Accelerate")
                                ),
                                Controller.doAction(
                                        (rg, ds) -> List.of( new DataStateUpdate(accel, - BRAKE),
                                                new DataStateUpdate(timer_V, TIMER)),
                                        registry.reference("Decelerate"))
                        ),
                        Controller.ifThenElse(
                                DataState.greaterThan(gap,0),
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(accel, ACCELERATION),
                                                new DataStateUpdate(timer_V, TIMER)),
                                        registry.reference("Accelerate")
                                ),
                                Controller.doAction(
                                        (rg,ds)-> List.of(new DataStateUpdate(accel,NEUTRAL),
                                                new DataStateUpdate(timer_V, TIMER)),
                                        registry.reference("Stop")
                                )
                        )
                )
        );

        registry.set("Accelerate",
                Controller.ifThenElse(
                        DataState.greaterThan(timer_V, 0),
                        Controller.doTick(registry.reference("Accelerate")),
                        registry.reference("Ctrl")
                )
        );

        registry.set("Decelerate",
                Controller.ifThenElse(
                        DataState.greaterThan(timer_V, 0),
                        Controller.doTick(registry.reference("Decelerate")),
                        registry.reference("Ctrl")
                )
        );

        registry.set("Stop",
                Controller.ifThenElse(
                        DataState.greaterThan(timer_V, 0),
                        Controller.doTick(registry.reference("Stop")),
                        Controller.doAction((rg,ds)-> List.of(new DataStateUpdate(timer_V, TIMER)),
                                registry.reference("Stop")
                        )
                )
        );



        return registry.reference("Ctrl");

    }


    // ENVIRONMENT EVOLUTION

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double travel = Math.max(state.get(accel)/2 + state.get(p_speed),0);
        double new_timer_V = state.get(timer_V) - 1;
        double new_p_speed;
        if (state.get(accel) == NEUTRAL) {
            new_p_speed = Math.max(0.0,state.get(p_speed)-ACCELERATION);
        } else {
            new_p_speed = Math.min(MAX_SPEED, Math.max(0, state.get(p_speed) + state.get(accel)));
        }
        double token = rg.nextDouble();
        double new_s_speed;
        if (token < 0.5){
            new_s_speed = new_p_speed + rg.nextDouble()*0.5;
        } else {
            new_s_speed = new_p_speed - rg.nextDouble()*0.5;
        }
        double new_p_distance = state.get(p_distance) - travel;
        updates.add(new DataStateUpdate(timer_V, new_timer_V));
        updates.add(new DataStateUpdate(p_speed, new_p_speed));
        updates.add(new DataStateUpdate(p_distance, new_p_distance));
        if(new_timer_V == 0) {
            double new_braking_distance = (new_s_speed * new_s_speed +
                    (ACCELERATION + BRAKE) * (ACCELERATION * TIMER * TIMER +
                            2 * new_s_speed * TIMER)) / (2 * BRAKE);
            double new_gap = new_p_distance - new_braking_distance;
            updates.add(new DataStateUpdate(s_speed, new_s_speed));
            updates.add(new DataStateUpdate(braking_distance, new_braking_distance));
            updates.add(new DataStateUpdate(gap, new_gap));
        }
        return updates;
    }


    // INITIALISATION OF DATA STATE

    public static DataState getInitialState( ) {
        Map<Integer, Double> values = new HashMap<>();

        values.put(timer_V, (double) 0);
        values.put(p_speed, INIT_SPEED);
        values.put(s_speed, INIT_SPEED);
        values.put(p_distance, INIT_DISTANCE);
        values.put(accel, NEUTRAL);
        double init_braking_distance = (INIT_SPEED * INIT_SPEED + (ACCELERATION + BRAKE) * (ACCELERATION * TIMER * TIMER +
                2 * INIT_SPEED * TIMER))/(2 * BRAKE);
        double init_gap = INIT_DISTANCE - init_braking_distance;
        values.put(braking_distance, init_braking_distance);
        values.put(gap, init_gap);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }

}
