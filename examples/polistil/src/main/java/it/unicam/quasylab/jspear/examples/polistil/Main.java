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

package it.unicam.quasylab.jspear.examples.polistil;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.GenerativeChoiceController;
import it.unicam.quasylab.jspear.controller.ParallelController;
import it.unicam.quasylab.jspear.distl.*;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.*;

public class Main {
    public final static String[] VARIABLES =
            new String[]{"my_x", "my_y", "my_theta", "my_speed", "curve_theta", "wp_i", "out", "back",
                    "your_x", "your_y", "your_theta", "your_speed", "your_curve_theta", "wp_j", "you_out", "you_back"
            };

    public final static double FULL = 4.0;
    public final static double CURVE = 3.0;
    public final static double MINIMAL = 2.0;
    public final static double NEUTRAL = 0.0;
    public final static double TIMER = 0.5*Math.PI/(9*CURVE);
    public final static double BACK_ON_TRACK = 5;
    public final static double INIT_X = 0.0;
    public final static double INIT_Y = 0.0;
    public final static double INIT_THETA = (3.0/4)*Math.PI;
    public final static double CX = 0.0;
    public final static double CY = Math.sqrt(2);
    public final static double RAD = 180/Math.PI;
    public final static double SHIFT_X = 0.3;
    public final static double SHIFT_Y = 0.0;

    private static final int H = 500;

    private static final int my_x = 0; // current position, first coordinate
    private static final int my_y = 1; // current position, second coordinate
    private static final int my_theta = 2; // direction on lanes
    private static final int my_speed = 3; // speed
    private static final int curve_theta = 4; // angle covered while curving
    private static final int wp_i = 5;
    private static final int out = 6;
    private static final int back = 7;

    private static final int your_x = 8; // current position, first coordinate
    private static final int your_y = 9; // current position, second coordinate
    private static final int your_theta = 10; // direction on lanes
    private static final int your_speed = 11; // speed
    private static final int your_curve_theta = 12; // angle covered while curving
    private static final int wp_j = 13;
    private static final int you_out = 14;
    private static final int you_back = 15;

    private static final int NUMBER_OF_VARIABLES = 16;

    public static void main(String[] args) throws IOException {
        try {

            RandomGenerator rand = new DefaultRandomGenerator();
            Controller car_1 = getCar_1();
            Controller car_2 = getCar_2();
            Controller race = new ParallelController(car_1,car_2);
            Controller siblings = new ParallelController(sib_1(),sib_2());
            DataState state = getInitialState();
            ControlledSystem system = new ControlledSystem(car_1, (rg, ds) -> ds.apply(EnvironmentSingle(rg, ds)), state);
            ControlledSystem race_system = new ControlledSystem(race, (rg, ds) -> ds.apply(EnvironmentRace(rg, ds)), state);
            ControlledSystem sib_system = new ControlledSystem(siblings, (rg, ds) -> ds.apply(EnvironmentRace(rg, ds)), state);
            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, 100);
            EvolutionSequence race_sequence = new EvolutionSequence(rand,rg -> race_system, 100);
            EvolutionSequence sib_sequence = new EvolutionSequence(rand,rg -> sib_system, 100);

            /*
            USING THE SIMULATOR
             */

            ArrayList<String> L = new ArrayList<>();
            L.add("x");
            L.add("y");
            L.add("theta");
            L.add("speed");
            L.add("curve");
            L.add(" waypoint");

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(my_x));
            F.add(ds->ds.get(my_y));
            F.add(ds->ds.get(my_theta));
            F.add(ds->ds.get(my_speed));
            F.add(ds->ds.get(curve_theta));
            F.add(ds->ds.get(wp_i));

            printLData(rand,L,F,system,H,1);

            ArrayList<DataStateExpression> V = new ArrayList<>();
            V.add(ds->ds.get(my_x));
            V.add(ds->ds.get(my_y));
            V.add(ds->ds.get(your_x));
            V.add(ds->ds.get(your_y));
            V.add(ds->ds.get(my_speed));
            V.add(ds->ds.get(your_speed));


            double[][] single_trajectory = new double[H][2];
            double[][] average_trajectory = new double[H][2];
            double[][] race_single_1 = new double[H][2];
            double[][] race_single_2 = new double[H][2];
            double[][] race_average_1 = new double[H][2];
            double[][] race_average_2 = new double[H][2];

            /*
            System.out.println("Simulating one car only");

            System.out.println("single trajectory");

            double[][] data = SystemState.sample(rand, V, system, H, 1);
            for (int i = 0; i<H; i++){
                single_trajectory[i][0] = data[i][0];
                single_trajectory[i][1] = data[i][1];
            }
            Util.writeToCSV("./JF_trajectory_single.csv",single_trajectory);

            System.out.println("average trajectory");

            double[][] data_av = SystemState.sample(rand, V, system, H, 100000);
            for (int i = 0; i<H; i++){
                average_trajectory[i][0] = data_av[i][0];
                average_trajectory[i][1] = data_av[i][1];
            }
            Util.writeToCSV("./JF_trajectory_average.csv",average_trajectory);

            System.out.println("Simulating race");

            System.out.println("single trajectory");

            double[][] data_r = SystemState.sample(rand, V, race_system, H, 1);
            for (int i = 0; i<H; i++){
                race_single_1[i][0] = data_r[i][0];
                race_single_1[i][1] = data_r[i][1];
                race_single_2[i][0] = data_r[i][2];
                race_single_2[i][1] = data_r[i][3];
            }
            Util.writeToCSV("./JF_race_single_1.csv",race_single_1);
            Util.writeToCSV("./JF_race_single_2.csv",race_single_2);

            System.out.println("average trajectory");

            double[][] data_rav = SystemState.sample(rand, V, race_system, H, 100000);
            for (int i = 0; i<H; i++){
                race_average_1[i][0] = data_rav[i][0];
                race_average_1[i][1] = data_rav[i][1];
                race_average_2[i][0] = data_rav[i][2];
                race_average_2[i][1] = data_rav[i][3];
            }
            Util.writeToCSV("./JF_race_average_1.csv",race_average_1);
            Util.writeToCSV("./JF_race_average_2.csv",race_average_2);

            double[][] race_speed_1 = new double[150][1];
            double[][] race_speed_2 = new double[150][1];
            double[][] race_av_speed_1 = new double[H][1];
            double[][] race_av_speed_2 = new double[H][1];

            System.out.println("single speed");

            double[][] data_s = SystemState.sample(rand, V, race_system, 150, 1);
            for (int i = 0; i<150; i++){
                race_speed_1[i][0] = data_s[i][4];
                race_speed_2[i][0] = data_s[i][5];
            }
            Util.writeToCSV("./JF_race_speed_1.csv",race_speed_1);
            Util.writeToCSV("./JF_race_speed_2.csv",race_speed_2);

            System.out.println("average speed");

            double[][] data_as = SystemState.sample(rand, V, race_system, H, 100000);
            for (int i = 0; i<H; i++){
                race_av_speed_1[i][0] = data_as[i][4];
                race_av_speed_2[i][0] = data_as[i][5];
            }
            Util.writeToCSV("./JF_race_av_speed_1.csv",race_av_speed_1);
            Util.writeToCSV("./JF_race_av_speed_2.csv",race_av_speed_2);

            System.out.println("average out");

            ArrayList<DataStateExpression> O = new ArrayList<>();
            O.add(ds->ds.get(out));
            O.add(ds->ds.get(you_out));

            double[][] race_out_1 = new double[H][1];
            double[][] race_out_2 = new double[H][1];

            double[][] data_o = SystemState.sample(rand, O, race_system, H, 100000);
            for (int i = 0; i<H; i++){
                race_out_1[i][0] = data_o[i][0];
                race_out_2[i][0] = data_o[i][1];
            }
            Util.writeToCSV("./JF_race_out_1.csv",race_out_1);
            Util.writeToCSV("./JF_race_out_2.csv",race_out_2);

            System.out.println("Simulating race against sibling");

            double[][] sib_out_1 = new double[H][1];
            double[][] sib_out_2 = new double[H][1];

            double[][] data_so = SystemState.sample(rand, O, sib_system, H, 100000);
            for (int i = 0; i<H; i++){
                sib_out_1[i][0] = data_so[i][0];
                sib_out_2[i][0] = data_so[i][1];
            }
            Util.writeToCSV("./JF_sib_out_1.csv",sib_out_1);
            Util.writeToCSV("./JF_sib_out_2.csv",sib_out_2);


             */


            //ANALYSIS WITH DisTL

            DataStateFunction mu_out_1 = (rg, ds) -> ds.apply(getDiracOut1(rg, ds));
            DataStateFunction mu_out_2 = (rg, ds) -> ds.apply(getDiracOut2(rg, ds));

            double eta_out = 0.01;

            DisTLFormula phi_out_1 = new AlwaysDisTLFormula(
                    new TargetDisTLFormula(
                            mu_out_1,
                            ds -> ds.get(out),
                            eta_out
                    ),
                    0,
                    H
            );

            DisTLFormula phi_out_2 = new AlwaysDisTLFormula(
                    new TargetDisTLFormula(
                            mu_out_2,
                            ds -> ds.get(you_out),
                            eta_out
                    ),
                    0,
                    H
            );

            double value = new DoubleSemanticsVisitor().eval(phi_out_1).eval(10, 0, sequence);

            System.out.println("Robustness of the first car, in 0, wrt phi_out_1: "+value/eta_out);

            double value_1 = new DoubleSemanticsVisitor().eval(phi_out_1).eval(10, 0, race_sequence);

            System.out.println("Robustness of the first car, in 0, wrt phi_out_1, when racing: "+value_1/eta_out);

            double value_2 = new DoubleSemanticsVisitor().eval(phi_out_2).eval(10, 0, race_sequence);

            System.out.println("Robustness of the second car, in 0, wrt phi_out_2, when racing: "+value_2/eta_out);

            double value_1s = new DoubleSemanticsVisitor().eval(phi_out_1).eval(10, 0, sib_sequence);

            System.out.println("Robustness of the first car, in 0, wrt phi_out_1, when racing against sibling: "+value_1s);

            double value_2s = new DoubleSemanticsVisitor().eval(phi_out_2).eval(10, 0, sib_sequence);

            System.out.println("Robustness of the second car, in 0, wrt phi_out_2, when racing against sibling: "+value_2s);

            DataStateFunction mu_full_1 = (rg, ds) -> ds.apply(getDiracSpeed1(rg, ds));
            DataStateFunction mu_full_2 = (rg, ds) -> ds.apply(getDiracSpeed2(rg, ds));
            DataStateFunction mu_gauss_x1 = (rg, ds) -> ds.apply(getGaussx1(rg, ds));
            DataStateFunction mu_gauss_x2 = (rg, ds) -> ds.apply(getGaussx2(rg, ds));

            double eta_speed = 0.1;
            double eta_pos = 0.2;

            DisTLFormula phi_speed_1 = new UntilDisTLFormula(
                    new TargetDisTLFormula(
                            mu_full_1,
                            ds -> ds.get(my_speed),
                            eta_speed
                    ),
                    1,
                    10,
                    new TargetDisTLFormula(
                            mu_gauss_x1,
                            ds -> ds.get(my_x),
                            eta_pos
                    )
            );

            DisTLFormula phi_speed_2 = new UntilDisTLFormula(
                    new TargetDisTLFormula(
                            mu_full_2,
                            ds -> ds.get(your_speed),
                            eta_speed
                    ),
                    1,
                    10,
                    new TargetDisTLFormula(
                            mu_gauss_x2,
                            ds -> ds.get(your_x),
                            eta_pos
                    )
            );

            double value_u1 = new DoubleSemanticsVisitor().eval(phi_speed_1).eval(10, 0, race_sequence);

            System.out.println("Robustness of the first car, in 0, wrt phi_speed_1, when racing: "+value_u1);

            double value_u2 = new DoubleSemanticsVisitor().eval(phi_speed_2).eval(10, 0, race_sequence);

            System.out.println("Robustness of the second car, in 0, wrt phi_speed_2, when racing: "+value_u2);

            double value_u1s = new DoubleSemanticsVisitor().eval(phi_speed_1).eval(10, 0, sib_sequence);

            System.out.println("Robustness of the first car, in 0, wrt phi_speed_1, when racing against sibling: "+value_u1s);

            double value_u2s = new DoubleSemanticsVisitor().eval(phi_speed_2).eval(10, 0, sib_sequence);

            System.out.println("Robustness of the second car, in 0, wrt phi_speed_2, when racing against sibling: "+value_u2s);




        } catch (RuntimeException e) {
            e.printStackTrace();
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

    public static List<DataStateUpdate> getDiracOut1(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(out, 0.0));
        return updates;
    }

    public static List<DataStateUpdate> getDiracOut2(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(you_out, 0.0));
        return updates;
    }

    public static List<DataStateUpdate> getDiracSpeed1(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(my_speed, FULL));
        return updates;
    }

    public static List<DataStateUpdate> getDiracSpeed2(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(your_speed, FULL));
        return updates;
    }

    public static List<DataStateUpdate> getGaussx1(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        double sign_x;
        if (state.get(my_x) >= 0){
            sign_x = 1;
        } else {
            sign_x = -1;
        }
        updates.add(new DataStateUpdate(my_x, rg.nextGaussian()*0.1 + sign_x*(Math.sqrt(2)/2 - MINIMAL*TIMER*Math.cos(state.get(my_theta)))));
        return updates;
    }

    public static List<DataStateUpdate> getGaussx2(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        double sign_x;
        if (state.get(your_x) >= 0){
            sign_x = 1;
        } else {
            sign_x = -1;
        }
        updates.add(new DataStateUpdate(your_x, rg.nextGaussian()*0.1 + SHIFT_X + sign_x*(Math.sqrt(2)/2 - MINIMAL*TIMER*Math.cos(state.get(your_theta)))));
        return updates;
    }



    // CONTROLLER OF VEHICLE

    public static Controller getCar_1() {

        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Ctrl",
                Controller.ifThenElse(
                        DataState.equalsTo(out,1.0),
                        Controller.doAction(
                                (rg, ds) -> List.of(new DataStateUpdate(my_speed,NEUTRAL),new DataStateUpdate(back,BACK_ON_TRACK)),
                                registry.reference("Stop")
                        ),
                        Controller.ifThenElse(
                                (rg, ds) -> ds.get(wp_i)%4==0 || ds.get(wp_i)%4==2,
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(my_speed,
                                                Math.max(MINIMAL,Math.min(FULL,
                                                        (Math.sqrt(2)/2 - Math.abs(ds.get(my_x)))/(TIMER*Math.abs(Math.cos(ds.get(my_theta)))))))),
                                        registry.reference("Ctrl")
                                ),
                                new GenerativeChoiceController(
                                        1.0/3,
                                        Controller.doAction(
                                                (rg, ds) -> List.of(new DataStateUpdate(my_speed, 2.7)),
                                                registry.reference("Ctrl")
                                        ),
                                        new GenerativeChoiceController(
                                                1.0/2,
                                                Controller.doAction(
                                                        (rg, ds) -> List.of(new DataStateUpdate(my_speed, 2.8)),
                                                        registry.reference("Ctrl")
                                                ),
                                                Controller.doAction(
                                                        (rg, ds) -> List.of(new DataStateUpdate(my_speed, 2.9)),
                                                        registry.reference("Ctrl")
                                                )
                                        )
                                )
                        )
                )
        );

        registry.set("Stop",
                Controller.ifThenElse(
                        DataState.greaterThan(back,0.0),
                        Controller.doTick(registry.reference("Stop")),
                        registry.reference("Ctrl")
                )
        );

        return registry.reference("Ctrl");
    }

    public static Controller getCar_2() {

        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Ctrl2",
                Controller.ifThenElse(
                        DataState.equalsTo(you_out,1.0),
                        Controller.doAction(
                                (rg, ds) -> List.of(new DataStateUpdate(your_speed,NEUTRAL),new DataStateUpdate(you_back,BACK_ON_TRACK)),
                                registry.reference("Stop2")
                        ),
                        Controller.ifThenElse(
                                (rg, ds) -> ds.get(wp_j)%4==0 || ds.get(wp_j)%4==2,
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(your_speed,
                                                Math.max(MINIMAL,Math.min(FULL,
                                                        (Math.sqrt(2)/2 - SHIFT_X - Math.abs(ds.get(your_x)))/(TIMER*Math.abs(Math.cos(ds.get(your_theta)))))))),
                                        registry.reference("Ctrl2")
                                ),
                                new GenerativeChoiceController(
                                        1.0/3,
                                        Controller.doAction(
                                                (rg, ds) -> List.of(new DataStateUpdate(your_speed, 2.7)),
                                                registry.reference("Ctrl2")
                                        ),
                                        new GenerativeChoiceController(
                                                1.0/2,
                                                Controller.doAction(
                                                        (rg, ds) -> List.of(new DataStateUpdate(your_speed, 2.8)),
                                                        registry.reference("Ctrl2")
                                                ),
                                                Controller.doAction(
                                                        (rg, ds) -> List.of(new DataStateUpdate(your_speed, 2.9)),
                                                        registry.reference("Ctrl2")
                                                )
                                        )
                                )
                        )
                )
        );

        registry.set("Stop2",
                Controller.ifThenElse(
                        DataState.greaterThan(you_back,0.0),
                        Controller.doTick(registry.reference("Stop2")),
                        registry.reference("Ctrl2")
                )
        );

        return registry.reference("Ctrl2");
    }

    // SIBLINGS MODE

    public static Controller sib_1() {

        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Sib",
                Controller.ifThenElse(
                        DataState.equalsTo(out,1.0),
                        Controller.doAction(
                                (rg, ds) -> List.of(new DataStateUpdate(my_speed,NEUTRAL),new DataStateUpdate(back,BACK_ON_TRACK)),
                                registry.reference("StopS")
                        ),
                        Controller.ifThenElse(
                                (rg, ds) -> ds.get(wp_i)%4==0 || ds.get(wp_i)%4==2,
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(my_speed,
                                                Math.max(MINIMAL,Math.min(FULL,
                                                        (Math.sqrt(2)/2 - Math.abs(ds.get(my_x)))/(TIMER*Math.abs(Math.cos(ds.get(my_theta)))))))),
                                        registry.reference("Sib")
                                ),
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(my_speed, CURVE)),
                                        registry.reference("Sib")
                                )
                        )
                )
        );

        registry.set("StopS",
                Controller.ifThenElse(
                        DataState.greaterThan(back,0.0),
                        Controller.doTick(registry.reference("StopS")),
                        registry.reference("Sib")
                )
        );

        return registry.reference("Sib");
    }

    public static Controller sib_2() {

        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Sib2",
                Controller.ifThenElse(
                        DataState.equalsTo(you_out,1.0),
                        Controller.doAction(
                                (rg, ds) -> List.of(new DataStateUpdate(your_speed,NEUTRAL),new DataStateUpdate(you_back,BACK_ON_TRACK)),
                                registry.reference("StopS2")
                        ),
                        Controller.ifThenElse(
                                (rg, ds) -> ds.get(wp_j)%4==0 || ds.get(wp_j)%4==2,
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(your_speed,
                                                Math.max(MINIMAL,Math.min(FULL,
                                                        (Math.sqrt(2)/2 - SHIFT_X - Math.abs(ds.get(your_x)))/(TIMER*Math.abs(Math.cos(ds.get(your_theta)))))))),
                                        registry.reference("Sib2")
                                ),
                                Controller.doAction(
                                        (rg, ds) -> List.of(new DataStateUpdate(your_speed, CURVE)),
                                        registry.reference("Sib2")
                                )
                        )
                )
        );

        registry.set("StopS2",
                Controller.ifThenElse(
                        DataState.greaterThan(you_back,0.0),
                        Controller.doTick(registry.reference("StopS2")),
                        registry.reference("Sib2")
                )
        );

        return registry.reference("Sib2");
    }




    // ENVIRONMENT EVOLUTION

    public static List<DataStateUpdate> EnvironmentSingle(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double i = state.get(wp_i);
        double speed;

        if(state.get(my_speed) == FULL || state.get(my_speed) == NEUTRAL){
            speed = state.get(my_speed);
        } else {
            speed = Math.max(0,Math.min(FULL,state.get(my_speed) + rg.nextDouble()*0.1-0.05));
        }

        updates.add(new DataStateUpdate(my_speed,speed));

        double new_x;
        double new_y;
        double run_theta = speed*TIMER;
        double new_theta;

        if (i % 4 == 0){
            double partial_x = state.get(my_x) + speed*TIMER*Math.cos(state.get(my_theta));
            double partial_y = state.get(my_y) + speed*TIMER*Math.sin(state.get(my_theta));
            if(partial_x < -Math.sqrt(2)/2){
                double extra = Math.abs(partial_x)-Math.sqrt(2)/2;
                double new_timer = Math.abs(extra/(speed*Math.cos(state.get(my_theta))));
                double extra_theta = speed*new_timer;
                new_theta = state.get(curve_theta)-extra_theta;
                new_x = Math.cos(new_theta)+CX;
                new_y = Math.sin(new_theta)+CY;
                updates.add(new DataStateUpdate(wp_i,i+1));
                updates.add(new DataStateUpdate(my_theta,(5.0/4)*Math.PI));
                if (speed > CURVE){
                    updates.add(new DataStateUpdate(out,1.0));
                }
            } else {
                new_x = partial_x;
                new_y = partial_y;
                new_theta = state.get(curve_theta);
            }
        } else if (i % 4 == 2) {
            double partial_x = state.get(my_x) + speed*TIMER*Math.cos(state.get(my_theta));
            double partial_y = state.get(my_y) + speed*TIMER*Math.sin(state.get(my_theta));
            if(partial_x < -Math.sqrt(2)/2){
                double extra = Math.abs(partial_x)-Math.sqrt(2)/2;
                double new_timer = Math.abs(extra/(speed*Math.cos(state.get(my_theta))));
                double extra_theta = speed*new_timer;
                new_theta = state.get(curve_theta)+extra_theta;
                new_x = Math.cos(new_theta)+CX;
                new_y = Math.sin(new_theta)-CY;
                updates.add(new DataStateUpdate(wp_i,i+1));
                updates.add(new DataStateUpdate(my_theta,(3.0/4)*Math.PI));
                if (speed > CURVE){
                    updates.add(new DataStateUpdate(out,1.0));
                }
            } else {
                new_x = partial_x;
                new_y = partial_y;
                new_theta = state.get(curve_theta);
            }
        } else if (i % 4 == 1) {
            if (speed > CURVE){
                updates.add(new DataStateUpdate(out,1.0));
            }
            double partial_theta = state.get(curve_theta) - run_theta;
            double partial_x = Math.cos(partial_theta) + CX;
            double partial_y = Math.sin(partial_theta) + CY;
            if (partial_x >= 0 && partial_x < Math.sqrt(2)/2 && partial_y < Math.sqrt(2)/2) {
                double extra_theta = Math.abs(partial_theta)-Math.PI/4;
                double new_timer = extra_theta/(speed*RAD);
                double done_theta = state.get(curve_theta) - run_theta + extra_theta;
                new_x = Math.cos(done_theta) + CX + speed*new_timer*Math.cos(state.get(my_theta));
                new_y = Math.sin(done_theta)+ CY + speed*new_timer*Math.sin(state.get(my_theta));
                new_theta = (3.0/4)*Math.PI;
                updates.add(new DataStateUpdate(wp_i,i+1));
            } else {
                new_x = partial_x;
                new_y = partial_y;
                new_theta = partial_theta;
            }
        } else {
            if (speed > CURVE){
                updates.add(new DataStateUpdate(out,1.0));
            }
            double partial_theta = state.get(curve_theta) + run_theta;
            double partial_x = Math.cos(partial_theta) + CX;
            double partial_y = Math.sin(partial_theta) - CY;
            if (partial_x >= 0 && partial_x < Math.sqrt(2)/2 && partial_y > -Math.sqrt(2)/2) {
                double extra_theta = Math.abs(Math.PI/4 - partial_theta);
                double new_timer = extra_theta/(speed*RAD);
                double done_theta = state.get(curve_theta) + run_theta - extra_theta;
                new_x = Math.cos(done_theta) + CX + speed*new_timer*Math.cos(state.get(my_theta));
                new_y = Math.sin(done_theta) - CY + speed*new_timer*Math.sin(state.get(my_theta));
                new_theta = (5.0/4)*Math.PI;
                updates.add(new DataStateUpdate(wp_i,i+1));
            } else {
                new_x = partial_x;
                new_y = partial_y;
                new_theta = partial_theta;
            }
        }

        updates.add(new DataStateUpdate(my_x,new_x));
        updates.add(new DataStateUpdate(my_y,new_y));
        updates.add(new DataStateUpdate(curve_theta,new_theta));

        if(state.get(back)==0){
            updates.add(new DataStateUpdate(out,0.0));
        } else {
            updates.add(new DataStateUpdate(back, state.get(back)-1));
        }

        return updates;
    }

    public static List<DataStateUpdate> EnvironmentRace(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        if (state.get(out)==1.0){
            if (state.get(back) -1 == 0) {
                updates.add(new DataStateUpdate(out, 0.0));
            }
            updates.add(new DataStateUpdate(back, state.get(back) - 1));
        } else {
            double i = state.get(wp_i);
            double speed;

            if (state.get(my_speed) == FULL || state.get(my_speed) == NEUTRAL) {
                speed = state.get(my_speed);
            } else {
                speed = Math.max(0, Math.min(FULL, state.get(my_speed) + rg.nextDouble() * 0.1 - 0.05));
            }

            updates.add(new DataStateUpdate(my_speed, speed));

            double new_x;
            double new_y;
            double run_theta = speed * TIMER;
            double new_theta;

            if (i % 4 == 0) {
                double partial_x = state.get(my_x) + speed * TIMER * Math.cos(state.get(my_theta));
                double partial_y = state.get(my_y) + speed * TIMER * Math.sin(state.get(my_theta));
                if (partial_x < -Math.sqrt(2) / 2) {
                    if (speed > CURVE) {
                        new_x = -Math.sqrt(2) / 2;
                        new_y = Math.sqrt(2) / 2;
                        new_theta = state.get(curve_theta);
                        updates.add(new DataStateUpdate(wp_i,i+1));
                        updates.add(new DataStateUpdate(my_theta, (5.0 / 4) * Math.PI));
                        updates.add(new DataStateUpdate(out, 1.0));
                    } else {
                        double extra = Math.abs(partial_x) - Math.sqrt(2) / 2;
                        double new_timer = Math.abs(extra / (speed * Math.cos(state.get(my_theta))));
                        double extra_theta = speed * new_timer;
                        new_theta = state.get(curve_theta) - extra_theta;
                        new_x = Math.cos(new_theta) + CX;
                        new_y = Math.sin(new_theta) + CY;
                        updates.add(new DataStateUpdate(wp_i, i + 1));
                        updates.add(new DataStateUpdate(my_theta, (5.0 / 4) * Math.PI));
                    }
                } else{
                    new_x = partial_x;
                    new_y = partial_y;
                    new_theta = state.get(curve_theta);
                }
            } else if (i % 4 == 2) {
                double partial_x = state.get(my_x) + speed * TIMER * Math.cos(state.get(my_theta));
                double partial_y = state.get(my_y) + speed * TIMER * Math.sin(state.get(my_theta));
                if (partial_x < -Math.sqrt(2) / 2) {
                    if (speed > CURVE) {
                        new_x = -Math.sqrt(2) / 2;
                        new_y = -Math.sqrt(2) / 2;
                        new_theta = state.get(curve_theta);
                        updates.add(new DataStateUpdate(wp_i,i+1));
                        updates.add(new DataStateUpdate(my_theta, (3.0 / 4) * Math.PI));
                        updates.add(new DataStateUpdate(out, 1.0));
                    } else {
                        double extra = Math.abs(partial_x) - Math.sqrt(2) / 2;
                        double new_timer = Math.abs(extra / (speed * Math.cos(state.get(my_theta))));
                        double extra_theta = speed * new_timer;
                        new_theta = state.get(curve_theta) + extra_theta;
                        new_x = Math.cos(new_theta) + CX;
                        new_y = Math.sin(new_theta) - CY;
                        updates.add(new DataStateUpdate(wp_i, i + 1));
                        updates.add(new DataStateUpdate(my_theta, (3.0 / 4) * Math.PI));
                    }
                } else {
                    new_x = partial_x;
                    new_y = partial_y;
                    new_theta = state.get(curve_theta);
                }
            } else if (i % 4 == 1) {
                if (speed > CURVE) {
                    new_x = state.get(my_x);
                    new_y = state.get(my_y);
                    new_theta = state.get(curve_theta);
                    updates.add(new DataStateUpdate(out, 1.0));
                } else {
                    double partial_theta = state.get(curve_theta) - run_theta;
                    double partial_x = Math.cos(partial_theta) + CX;
                    double partial_y = Math.sin(partial_theta) + CY;
                    if (partial_x >= 0 && partial_x < Math.sqrt(2) / 2 && partial_y < Math.sqrt(2) / 2) {
                        double extra_theta = Math.abs(partial_theta) - Math.PI / 4;
                        double new_timer = extra_theta / (speed * RAD);
                        double done_theta = state.get(curve_theta) - run_theta + extra_theta;
                        new_x = Math.cos(done_theta) + CX + speed * new_timer * Math.cos(state.get(my_theta));
                        new_y = Math.sin(done_theta) + CY + speed * new_timer * Math.sin(state.get(my_theta));
                        new_theta = (3.0 / 4) * Math.PI;
                        updates.add(new DataStateUpdate(wp_i, i + 1));
                    } else {
                        new_x = partial_x;
                        new_y = partial_y;
                        new_theta = partial_theta;
                    }
                }
            } else {
                if (speed > CURVE) {
                    new_x = state.get(my_x);
                    new_y = state.get(my_y);
                    new_theta = state.get(curve_theta);
                    updates.add(new DataStateUpdate(out, 1.0));
                } else {
                    double partial_theta = state.get(curve_theta) + run_theta;
                    double partial_x = Math.cos(partial_theta) + CX;
                    double partial_y = Math.sin(partial_theta) - CY;
                    if (partial_x >= 0 && partial_x < Math.sqrt(2) / 2 && partial_y > -Math.sqrt(2) / 2) {
                        double extra_theta = Math.abs(Math.PI / 4 - partial_theta);
                        double new_timer = extra_theta / (speed * RAD);
                        double done_theta = state.get(curve_theta) + run_theta - extra_theta;
                        new_x = Math.cos(done_theta) + CX + speed * new_timer * Math.cos(state.get(my_theta));
                        new_y = Math.sin(done_theta) - CY + speed * new_timer * Math.sin(state.get(my_theta));
                        new_theta = (5.0 / 4) * Math.PI;
                        updates.add(new DataStateUpdate(wp_i, i + 1));
                    } else {
                        new_x = partial_x;
                        new_y = partial_y;
                        new_theta = partial_theta;
                    }
                }
            }

            updates.add(new DataStateUpdate(my_x, new_x));
            updates.add(new DataStateUpdate(my_y, new_y));
            updates.add(new DataStateUpdate(curve_theta, new_theta));

        }

        if (state.get(you_out)==1.0){
            if (state.get(you_back) -1 == 0) {
                updates.add(new DataStateUpdate(you_out, 0.0));
            }
            updates.add(new DataStateUpdate(you_back, state.get(you_back) - 1));
        } else {

            double j = state.get(wp_j);
            double y_speed;

            if (state.get(your_speed) == FULL || state.get(your_speed) == NEUTRAL) {
                y_speed = state.get(your_speed);
            } else {
                y_speed = Math.max(0, Math.min(FULL, state.get(your_speed) + rg.nextDouble() * 0.1 - 0.05));
            }

            updates.add(new DataStateUpdate(your_speed, y_speed));

            double new_y_x;
            double new_y_y;
            double run_y_theta = y_speed * TIMER;
            double new_y_theta;

            if (j % 4 == 0) {
                double partial_x = state.get(your_x) + y_speed * TIMER * Math.cos(state.get(your_theta));
                double partial_y = state.get(your_y) + y_speed * TIMER * Math.sin(state.get(your_theta));
                if (partial_x < -Math.sqrt(2)/2 + SHIFT_X) {
                    if (y_speed > CURVE) {
                        new_y_x = -Math.sqrt(2)/2 + SHIFT_X;
                        new_y_y = Math.sqrt(2)/2 + SHIFT_Y;
                        new_y_theta = state.get(your_curve_theta);
                        updates.add(new DataStateUpdate(wp_j,j+1));
                        updates.add(new DataStateUpdate(your_theta, (5.0 / 4) * Math.PI));
                        updates.add(new DataStateUpdate(you_out, 1.0));
                    } else {
                        double extra = Math.abs(partial_x) - Math.sqrt(2) / 2 + SHIFT_X;
                        double new_timer = Math.abs(extra / (y_speed * Math.cos(state.get(your_theta))));
                        double extra_theta = y_speed * new_timer;
                        new_y_theta = state.get(your_curve_theta) - extra_theta;
                        new_y_x = Math.cos(new_y_theta) + CX + SHIFT_X;
                        new_y_y = Math.sin(new_y_theta) + CY + SHIFT_Y;
                        updates.add(new DataStateUpdate(wp_j, j + 1));
                        updates.add(new DataStateUpdate(your_theta, (5.0 / 4) * Math.PI));
                    }
                } else {
                    new_y_x = partial_x;
                    new_y_y = partial_y;
                    new_y_theta = state.get(your_curve_theta);
                }
            } else if (j % 4 == 2) {
                double partial_x = state.get(your_x) + y_speed * TIMER * Math.cos(state.get(your_theta));
                double partial_y = state.get(your_y) + y_speed * TIMER * Math.sin(state.get(your_theta));
                if (partial_x < -Math.sqrt(2)/2 + SHIFT_X) {
                    if (y_speed > CURVE) {
                        new_y_x = -Math.sqrt(2)/2 + SHIFT_X;
                        new_y_y = -Math.sqrt(2)/2 + SHIFT_Y;
                        new_y_theta = state.get(your_curve_theta);
                        updates.add(new DataStateUpdate(wp_j,j+1));
                        updates.add(new DataStateUpdate(your_theta, (3.0 / 4) * Math.PI));
                        updates.add(new DataStateUpdate(you_out, 1.0));
                    } else {
                        double extra = Math.abs(partial_x) - Math.sqrt(2) / 2 + SHIFT_X;
                        double new_timer = Math.abs(extra / (y_speed * Math.cos(state.get(your_theta))));
                        double extra_theta = y_speed * new_timer;
                        new_y_theta = state.get(your_curve_theta) + extra_theta;
                        new_y_x = Math.cos(new_y_theta) + CX + SHIFT_X;
                        new_y_y = Math.sin(new_y_theta) - CY + SHIFT_Y;
                        updates.add(new DataStateUpdate(wp_j, j + 1));
                        updates.add(new DataStateUpdate(your_theta, (3.0 / 4) * Math.PI));
                    }
                } else {
                    new_y_x = partial_x;
                    new_y_y = partial_y;
                    new_y_theta = state.get(your_curve_theta);
                }
            } else if (j % 4 == 1) {
                if (y_speed > CURVE) {
                    new_y_x = state.get(your_x);
                    new_y_y = state.get(your_y);
                    new_y_theta = state.get(your_curve_theta);
                    updates.add(new DataStateUpdate(you_out, 1.0));
                } else {
                    double partial_theta = state.get(your_curve_theta) - run_y_theta;
                    double partial_x = Math.cos(partial_theta) + CX + SHIFT_X;
                    double partial_y = Math.sin(partial_theta) + CY + SHIFT_Y;
                    if (partial_x >= 0 + SHIFT_X && partial_x < Math.sqrt(2) / 2 + SHIFT_X && partial_y < Math.sqrt(2) / 2 + SHIFT_Y) {
                        double extra_theta = Math.abs(partial_theta) - Math.PI / 4;
                        double new_timer = extra_theta / (y_speed * RAD);
                        double done_theta = state.get(your_curve_theta) - run_y_theta + extra_theta;
                        new_y_x = Math.cos(done_theta) + CX + SHIFT_X + y_speed * new_timer * Math.cos(state.get(your_theta));
                        new_y_y = Math.sin(done_theta) + CY + SHIFT_Y + y_speed * new_timer * Math.sin(state.get(your_theta));
                        new_y_theta = (3.0 / 4) * Math.PI;
                        updates.add(new DataStateUpdate(wp_j, j + 1));
                    } else {
                        new_y_x = partial_x;
                        new_y_y = partial_y;
                        new_y_theta = partial_theta;
                    }
                }
            } else {
                if (y_speed > CURVE) {
                    new_y_x = state.get(your_x);
                    new_y_y = state.get(your_y);
                    new_y_theta = state.get(your_curve_theta);
                    updates.add(new DataStateUpdate(you_out, 1.0));
                } else {
                    double partial_theta = state.get(your_curve_theta) + run_y_theta;
                    double partial_x = Math.cos(partial_theta) + CX + SHIFT_X;
                    double partial_y = Math.sin(partial_theta) - CY + SHIFT_Y;
                    if (partial_x >= 0 + SHIFT_X && partial_x < Math.sqrt(2) / 2 + SHIFT_X && partial_y > -Math.sqrt(2) / 2 + SHIFT_Y) {
                        double extra_theta = Math.abs(Math.PI / 4 - partial_theta);
                        double new_timer = extra_theta / (y_speed * RAD);
                        double done_theta = state.get(your_curve_theta) + run_y_theta - extra_theta;
                        new_y_x = Math.cos(done_theta) + CX + SHIFT_X + y_speed * new_timer * Math.cos(state.get(your_theta));
                        new_y_y = Math.sin(done_theta) - CY + SHIFT_Y + y_speed * new_timer * Math.sin(state.get(your_theta));
                        new_y_theta = (5.0 / 4) * Math.PI;
                        updates.add(new DataStateUpdate(wp_j, j + 1));
                    } else {
                        new_y_x = partial_x;
                        new_y_y = partial_y;
                        new_y_theta = partial_theta;
                    }
                }
            }

            updates.add(new DataStateUpdate(your_x, new_y_x));
            updates.add(new DataStateUpdate(your_y, new_y_y));
            updates.add(new DataStateUpdate(your_curve_theta, new_y_theta));

        }

        return updates;

    }


    // INITIALISATION OF DATA STATE

    public static DataState getInitialState( ) {
        Map<Integer, Double> values = new HashMap<>();

        values.put(my_x, INIT_X);
        values.put(my_y, INIT_Y);
        values.put(my_theta, INIT_THETA);
        values.put(my_speed, NEUTRAL);
        values.put(curve_theta, (5.0/4)*Math.PI);
        values.put(wp_i,0.0);
        values.put(out,0.0);
        values.put(back,0.0);

        values.put(your_x, INIT_X+SHIFT_X);
        values.put(your_y, INIT_Y+SHIFT_Y);
        values.put(your_theta, INIT_THETA);
        values.put(your_speed, NEUTRAL);
        values.put(your_curve_theta, (5.0/4)*Math.PI);
        values.put(wp_j,0.0);
        values.put(you_out,0.0);
        values.put(you_back,0.0);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }


}

