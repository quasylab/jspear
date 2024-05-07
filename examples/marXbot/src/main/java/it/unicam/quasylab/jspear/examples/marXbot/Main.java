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

package it.unicam.quasylab.jspear.examples.marXbot;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.perturbation.*;
import it.unicam.quasylab.jspear.feedback.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.*;

public class Main {

    public final static String[] VARIABLES =
            new String[]{"p_speed", "s_speed", "p_distance", "accel", "timer_V", "braking_distance", "gap"
            };

    public final static double ACCELERATION = 0.25;
    public final static double BRAKE = 0.5;
    public final static double NEUTRAL = 0.0;
    public final static int TIMER = 2;
    public final static double INIT_SPEED = 0.0;
    public final static double MAX_SPEED = 3.0;
    public final static double MAX_SPEED_OFFSET = 1.0;
    public final static double INIT_X = 0.0;
    public final static double INIT_Y = 0.0;
    public final static double INIT_THETA = Math.PI/2;
    public final static double FINAL_X = 50.0;
    public final static double FINAL_Y = 30.0;
    public final static double[] WPx = {1,13,7,FINAL_X};
    public final static double[] WPy = {3,3,7,FINAL_Y};
    public final static double INIT_DISTANCE = Math.sqrt(Math.pow((WPx[0]-INIT_X),2) + Math.pow((WPy[0]-INIT_Y),2));

    private static final int H = 350;

    private static final int x = 0; // current position, first coordinate
    private static final int y = 1; // current position, second coordinate
    private static final int theta = 2; // current direction
    private static final int p_speed = 3; // physical speed
    private static final int s_speed = 4; // sensed speed
    private static final int p_distance = 5; // physical distance from the current target
    private static final int accel = 6; // acceleration
    private static final int timer_V = 7; // timer
    private static final int gap = 8; // difference between p_distance and the space required to stop when braking
    private static final int currentWP = 9; // current w point

    private static final int NUMBER_OF_VARIABLES = 10;
    private static final double SPEED_DIFFERENCE = 0.2;


    public static void main(String[] args) throws IOException {
        try {

            RandomGenerator rand = new DefaultRandomGenerator();

            Controller vehicle = getController();
            DataState state = getInitialState();
            ControlledSystem system = new ControlledSystem(vehicle, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);

            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, 100);

            Feedback feedback = new PersistentFeedback(new AtomicFeedback(0, sequence, Main::feedbackFunction));
            FeedbackSystem feedbackSystem = new FeedbackSystem(vehicle, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state, feedback);
            Perturbation perturbation = new PersistentPerturbation(new AtomicPerturbation(0, Main::slowerPerturbation));
            PerturbedSystem perturbedFeedbackSystem = new PerturbedSystem(feedbackSystem, perturbation);
            PerturbedSystem perturbedSystem = new PerturbedSystem(system, perturbation);


            ArrayList<String> L = new ArrayList<>();
            L.add("        x");
            L.add("        y");
            L.add("     theta");
            L.add("    p_speed");
            L.add("    s_speed");
            L.add("   distance");
            L.add("    gap ");

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(x));
            F.add(ds->ds.get(y));
            F.add(ds->ds.get(theta));
            F.add(ds->ds.get(p_speed));
            F.add(ds->ds.get(s_speed));
            F.add(ds->ds.get(p_distance));
            F.add(ds->ds.get(gap));
            F.add(ds->ds.get(currentWP));

            //printLData(rand,L,F,system,100,1);
            System.out.println(" ");
            System.out.println(" ");
            //printLData(rand,L,F,getIteratedSlowerPerturbation(),system,100,1);
            printLDataPar(rand,L,F,getIteratedSlowerPerturbation(),system,100,1);

        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }

    private static List<DataStateUpdate> feedbackFunction(RandomGenerator randomGenerator, DataState dataState, EvolutionSequence evolutionSequence) {
        int step = dataState.getStep();
        double meanSpeed = evolutionSequence.get(step).mean(ss -> ss.getDataState().get(s_speed));
        if (meanSpeed+SPEED_DIFFERENCE<dataState.get(s_speed)) {
            return List.of(new DataStateUpdate(accel, BRAKE));
        }
        if (meanSpeed-SPEED_DIFFERENCE>dataState.get(s_speed)) {
            return List.of(new DataStateUpdate(accel, ACCELERATION));
        }
        return List.of();
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

    private static void printLDataPar(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, Perturbation p, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, s, steps, size);
        double[][] datap = SystemState.sample(rg, F, p, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>  ", i);
            for (int j = 0; j < data[i].length-1; j++) {
                System.out.printf("%f  ", data[i][j]);
                System.out.printf("%f  ", datap[i][j]);
            }
            System.out.printf("%f  ", data[i][datap[i].length -1]);
            System.out.printf("%f\n", datap[i][datap[i].length -1]);

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

    // CONTROLLER OF VEHICLE

    public static Controller getController() {

        ControllerRegistry registry = new ControllerRegistry();

        registry.set("SetDir",
                Controller.ifThenElse(
                      DataState.greaterThan(gap,0),
                        Controller.doAction(
                                (rg, ds) -> List.of(new DataStateUpdate(theta, (WPx[(int)ds.get(currentWP)]==ds.get(x))?0:((WPx[(int)ds.get(currentWP)]<ds.get(x))?Math.PI:0)+Math.atan((WPy[(int)ds.get(currentWP)]-ds.get(y))/(WPx[(int)ds.get(currentWP)]-ds.get(x))))),
                                registry.reference("Ctrl")
                        ),
                        Controller.ifThenElse(
                                DataState.equalsTo(currentWP,WPx.length-1),
                                Controller.doAction((rg, ds) -> List.of(new DataStateUpdate(timer_V, TIMER)),
                                        registry.reference("Stop")),
                                Controller.doAction((rg, ds) -> List.of(new DataStateUpdate(currentWP, ds.get(currentWP)+1),
                                                new DataStateUpdate(theta, (WPx[(int)ds.get(currentWP)+1]==ds.get(x))?0:((WPx[(int)ds.get(currentWP)+1]<ds.get(x))?Math.PI:0)+Math.atan((WPy[(int)ds.get(currentWP)+1]-ds.get(y))/(WPx[(int)ds.get(currentWP)+1]-ds.get(x))))),
                                        registry.reference("Ctrl"))
                        )
                )
        );

        registry.set("Ctrl",
                Controller.ifThenElse(
                        DataState.greaterThan(s_speed, 0),
                        Controller.ifThenElse(
                                DataState.greaterThan(gap, 0),
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
                                        registry.reference("SetDir")
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



        return registry.reference("SetDir");

    }


    // ENVIRONMENT EVOLUTION

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double new_timer_V = state.get(timer_V) - 1; // timer is simply decremented
        double new_p_speed;
        if (state.get(accel) == NEUTRAL) {
            // the speed is updated according to acceleration
            new_p_speed = Math.max(0.0,state.get(p_speed)-ACCELERATION);
        } else {
            new_p_speed = Math.min(MAX_SPEED, Math.max(0, state.get(p_speed) + state.get(accel)));
        }
        //double token = rg.nextDouble();
        double new_s_speed = new_p_speed; // the sensed speed corresponds to the physical one in case of no perturbation
        double newX = state.get(x) + Math.cos(state.get(theta))*new_p_speed; // the position is updated according to
        double newY = state.get(y) + Math.sin(state.get(theta))*new_p_speed; // the physical speed
        //if (token < 0.5){
        //    new_s_speed = new_p_speed + rg.nextDouble()*0.5;
        //} else {
        //    new_s_speed = new_p_speed - rg.nextDouble()*0.5;
        //}
        double new_p_distance = Math.sqrt(Math.pow(WPx[(int)state.get(currentWP)]-newX,2) + Math.pow(WPy[(int)state.get(currentWP)]-newY,2));
        // the distance from the target is updated taking into account the new position
        updates.add(new DataStateUpdate(x,newX));
        updates.add(new DataStateUpdate(y,newY));
        updates.add(new DataStateUpdate(timer_V, new_timer_V));
        updates.add(new DataStateUpdate(p_speed, new_p_speed));
        updates.add(new DataStateUpdate(p_distance, new_p_distance));
        double new_braking_distance = (Math.pow(new_s_speed,2) + (ACCELERATION + BRAKE) * (ACCELERATION * Math.pow(TIMER,2) + 2 * new_s_speed * TIMER)) / (2 * BRAKE);
        // the braking distance is computed according to the uniformly accelerated motion law
        double new_gap = new_p_distance - new_braking_distance;
        updates.add(new DataStateUpdate(s_speed, new_s_speed));
        updates.add(new DataStateUpdate(gap, new_gap));
        return updates;
    }





    // INITIALISATION OF DATA STATE

    public static DataState getInitialState( ) {
        Map<Integer, Double> values = new HashMap<>();

        values.put(timer_V, 0.0);
        values.put(x, INIT_X);
        values.put(y, INIT_Y);
        values.put(theta, INIT_THETA);
        values.put(p_speed, INIT_SPEED);
        values.put(s_speed, INIT_SPEED);
        values.put(p_distance, INIT_DISTANCE);
        values.put(accel, NEUTRAL);
        double init_braking_distance = (Math.pow(INIT_SPEED,2) + (ACCELERATION + BRAKE) * (ACCELERATION * Math.pow(TIMER,2) + 2 * INIT_SPEED * TIMER))/(2 * BRAKE);
        double init_gap = INIT_DISTANCE - init_braking_distance;
        values.put(gap, init_gap);
        values.put(currentWP,0.0);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }


    // PERTURBATIONS

    private static  Perturbation getIteratedSlowerPerturbation() {
        return new AfterPerturbation(1, new IterativePerturbation(100, new AtomicPerturbation(TIMER - 2, Main::slowerPerturbation)));
    }

    private static DataState slowerPerturbation(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double offset = rg.nextDouble() * MAX_SPEED_OFFSET;
        double fake_speed = Math.max(0, state.get(p_speed) - offset);
        double fake_braking_distance = (Math.pow(fake_speed,2) + (ACCELERATION + BRAKE) * (ACCELERATION * Math.pow(TIMER,2) +
                2 * fake_speed * TIMER)) / (2 * BRAKE);
        double fake_gap = p_distance - fake_braking_distance;
        updates.add(new DataStateUpdate(s_speed, fake_speed));
        updates.add(new DataStateUpdate(gap, fake_gap));
        return state.apply(updates);
    }

}