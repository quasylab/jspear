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

package it.unicam.quasylab.jspear.examples.turtle;

import com.sun.jdi.BooleanValue;
import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.distance.*;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.perturbation.*;
import it.unicam.quasylab.jspear.feedback.*;
import it.unicam.quasylab.jspear.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.*;

public class Smart_hospital {

    public final static String[] VARIABLES =
            new String[]{"p_speed", "s_speed", "p_distance", "accel", "timer_V", "braking_distance", "gap"
            };

    public final static double ACCELERATION = 0.05;
    public final static double BRAKE = 0.1;
    public final static double NEUTRAL = 0.0;
    public final static int TIMER = 1;
    public final static double INIT_SPEED = 0.0;
    public final static double MAX_SPEED = 1.0;
    public final static double MAX_SPEED_WITH_MED = 0.5;
    private static final double SPEED_DIFFERENCE = 0.05;
    public final static double MAX_THETA_OFFSET = 0.1;
    private static final double DIR_DIFFERENCE = 0.001;
    public final static double INIT_X = 15.0;
    public final static double INIT_Y = 6.0;
    public final static double INIT_THETA = Math.PI/2;
    public final static double FINAL_X = 15.0;
    public final static double FINAL_Y = 6.0;
    public final static double[] WPx = {13,13,13,6,6,2,6,6,2,6,6,FINAL_X};
    public final static double[] WPy = {6,1,6,6,2,2,2,7,7,7,6,FINAL_Y};
    public final static double INIT_DISTANCE = Math.sqrt(Math.pow((WPx[0]-INIT_X),2) + Math.pow((WPy[0]-INIT_Y),2));

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
    private static final int previous_theta = 10;
    private static final int get_medicine = 11; // 0 if the robot is not transporting medicines. 1 if transporting. -1 if dropped.
    private static final int fail = 12; // 0 if the robot delivered correctly the medicines. 1 otherwise.
    private static final int flag = 13;

    private static final int NUMBER_OF_VARIABLES = 14;


    public static void main(String[] args) throws IOException {
        try {

            /*
            INITIAL CONFIGURATION
            */

            RandomGenerator rand = new DefaultRandomGenerator();

            Controller robot = getController();

            DataState state = getInitialState();

            ControlledSystem system = new ControlledSystem(robot, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);

            int sizeNominalSequence = 100;

            int scale = 500;

            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, sizeNominalSequence);

            Feedback feedbackDir = new PersistentFeedback(new AtomicFeedback(0, sequence, Smart_hospital::feedbackDirFunction));

            FeedbackSystem feedbackSystem = new FeedbackSystem(robot, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state, feedbackDir);

            EvolutionSequence feedbackSequence = new EvolutionSequence(rand, rg -> feedbackSystem, sizeNominalSequence);

            /*
            USING THE SIMULATOR
             */

            int N = 300;

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(x));
            F.add(ds->ds.get(y));
            F.add(ds->ds.get(theta));
            F.add(ds->ds.get(p_speed));
            F.add(ds->(int)ds.get(currentWP));
            F.add(ds->ds.get(previous_theta));
            F.add(ds->(int)ds.get(get_medicine));
            F.add(ds->(int)ds.get(fail));

            double[][] position_system = new double[N][2];
            double[][] position_perturbed_feedback_system = new double[N][2];

            double[][] get_med_system = new double[N][1];
            double[][] get_med_perturbed_feedback_system = new double[N][1];

            double[][] fail_system = new double[N][1];
            double[][] fail_perturbed_feedback_system = new double[N][1];

            double[][] data = SystemState.sample(rand, F, system, N, sizeNominalSequence);
            for (int i = 0; i<N; i++){
                position_system[i][0] = data[i][0];
                position_system[i][1] = data[i][1];
                get_med_system[i][0] = data[i][6];
                fail_system[i][0] = data[i][7];
            }
            Util.writeToCSV("./xy_nominal.csv",position_system);
            Util.writeToCSV("./get_med_nominal.csv",get_med_system);
            Util.writeToCSV("./fail_nominal.csv",fail_system);

            // Fixed k, changing off

            int k = 5;

            double[][] pfdata_100 = SystemState.sample(rand, F, ChangeDir(1.0,k), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_100[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_100[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_100[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_100[i][7];
            }
            Util.writeToCSV("./xy_feedback_100.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_100.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_100.csv",fail_perturbed_feedback_system);

            double[][] pfdata_125 = SystemState.sample(rand, F, ChangeDir(1.25,k), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_125[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_125[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_125[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_125[i][7];
            }
            Util.writeToCSV("./xy_feedback_125.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_125.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_125.csv",fail_perturbed_feedback_system);

            double[][] pfdata_150 = SystemState.sample(rand, F, ChangeDir(1.5,k), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_150[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_150[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_150[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_150[i][7];
            }
            Util.writeToCSV("./xy_feedback_150.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_150.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_150.csv",fail_perturbed_feedback_system);

            double[][] pfdata_175 = SystemState.sample(rand, F, ChangeDir(1.75,k), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_175[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_175[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_175[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_175[i][7];
            }
            Util.writeToCSV("./xy_feedback_175.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_175.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_175.csv",fail_perturbed_feedback_system);

            double[][] pfdata_2 = SystemState.sample(rand, F, ChangeDir(2.0,k), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_2[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_2[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_2[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_2[i][7];
            }
            Util.writeToCSV("./xy_feedback_2.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_2.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_2.csv",fail_perturbed_feedback_system);


            // Fixed off, changing k

            double[][] pfdata_5 = SystemState.sample(rand, F, ChangeDir(1.5,5), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_5[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_5[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_5[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_5[i][7];
            }
            Util.writeToCSV("./xy_feedback_5.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_5.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_5.csv",fail_perturbed_feedback_system);

            double[][] pfdata_10 = SystemState.sample(rand, F, ChangeDir(1.5,10), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_10[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_10[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_10[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_10[i][7];
            }
            Util.writeToCSV("./xy_feedback_10.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_10.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_10.csv",fail_perturbed_feedback_system);

            double[][] pfdata_15 = SystemState.sample(rand, F, ChangeDir(1.5,15), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_15[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_15[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_15[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_15[i][7];
            }
            Util.writeToCSV("./xy_feedback_15.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_15.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_15.csv",fail_perturbed_feedback_system);

            double[][] pfdata_20 = SystemState.sample(rand, F, ChangeDir(1.5,20), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_20[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_20[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_20[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_20[i][7];
            }
            Util.writeToCSV("./xy_feedback_20.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_20.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_20.csv",fail_perturbed_feedback_system);

            double[][] pfdata_25 = SystemState.sample(rand, F, ChangeDir(1.5,25), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_25[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_25[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_25[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_25[i][7];
            }
            Util.writeToCSV("./xy_feedback_25.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_25.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_25.csv",fail_perturbed_feedback_system);

            double[][] pfdata_30 = SystemState.sample(rand, F, ChangeDir(1.5,30), feedbackSystem, N, sizeNominalSequence*scale);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata_30[i][0];
                position_perturbed_feedback_system[i][1] = pfdata_30[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata_30[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata_30[i][7];
            }
            Util.writeToCSV("./xy_feedback_30.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback_30.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback_30.csv",fail_perturbed_feedback_system);

            /*
            EVALUATIONS OF DISTANCES
            */

            EvolutionSequence perturbedFeedbackSequence = feedbackSequence.apply(ChangeDir(1.25,5),0,scale);

            AtomicDistanceExpression distSpeed = new AtomicDistanceExpression(ds->(ds.get(p_speed)/MAX_SPEED), (v1, v2) -> Math.abs(v2-v1));

            double[][] direct_evaluation_atomic_distSpeed = new double[N][1];

            for (int i = 0; i<N; i++){
                direct_evaluation_atomic_distSpeed[i][0] = distSpeed.compute(i, sequence, perturbedFeedbackSequence);
            }

            Util.writeToCSV("./atomic_speed_nf.csv",direct_evaluation_atomic_distSpeed);


            AtomicDistanceExpression distTheta = new AtomicDistanceExpression(ds->(ds.get(theta)/(Math.PI*2)), (v1, v2) -> Math.abs(v2-v1));
            double[][] direct_evaluation_atomic_distTheta = new double[N][1];

            for (int i = 0; i<N; i++){
                direct_evaluation_atomic_distTheta[i][0] = distTheta.compute(i, sequence, perturbedFeedbackSequence);
            }

            Util.writeToCSV("./atomic_theta_nf.csv",direct_evaluation_atomic_distTheta);


            DistanceExpression flagDist = new AtomicDistanceExpression(ds->(ds.get(flag)), (v1, v2) -> Math.abs(v2-v1));
            DistanceExpression flag_left = new ThresholdDistanceExpression(flagDist,RelationOperator.LESS_THAN,1.0);
            DistanceExpression flag_right = new ThresholdDistanceExpression(flagDist,RelationOperator.GREATER_OR_EQUAL_THAN,1.0);
            DistanceExpression flagU = new UntilDistanceExpression(
                    flag_left,
                    40,
                    60,
                    flag_right
            );
            DistanceExpression flagM = new MaxIntervalDistanceExpression(
                    flagDist,
                    40,
                    60
            );

            double flagUntil = flagU.compute(0, sequence, perturbedFeedbackSequence);
            double flagMax = flagM.compute(0, sequence, perturbedFeedbackSequence);

            System.out.println(flagUntil);
            System.out.println(flagMax);

            /*
            MODEL CHECKING
            */

            k = 15;

            DistanceExpression failDist = new AtomicDistanceExpression(ds->(ds.get(fail)), (v1, v2) -> Math.abs(v2-v1));
            DistanceExpression failM = new MaxIntervalDistanceExpression(
                    failDist,
                    46-k,
                    251-k
            );
            int indexWF=0;
            double eta = 5;


            double[][] rob_125 = new double[11][2];
            double[][] rob_150 = new double[11][2];
            double[][] rob_175 = new double[11][2];

            for(int i = 0; i < 11 ; i++) {
                double new_eta = (eta + i) / 100;
                RobustnessFormula phi_125 = new AtomicRobustnessFormula(
                        ChangeDir(1.25, k),
                        failM,
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        new_eta
                );
                RobustnessFormula phi_fail_125 = new AlwaysRobustnessFormula(
                        phi_125,
                        0,
                        k - 1
                );
                Boolean value_125 = new BooleanSemanticsVisitor().eval(phi_fail_125).eval(N, 0, feedbackSequence);

                System.out.println(" ");
                System.out.println("\n off=1.25 evaluation at " + new_eta + ": " + value_125);
                rob_125[indexWF][1] = value_125?1:-1;
                rob_125[indexWF][0] = new_eta;
                indexWF++;
            }

            indexWF=0;
            for(int i = 0; i < 11 ; i++) {
                double new_eta = (eta + i) / 100;
                RobustnessFormula phi_150 = new AtomicRobustnessFormula(
                        ChangeDir(1.5, k),
                        failM,
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        new_eta
                );
                RobustnessFormula phi_fail_150 = new AlwaysRobustnessFormula(
                        phi_150,
                        0,
                        k - 1
                );
                Boolean value_150 = new BooleanSemanticsVisitor().eval(phi_fail_150).eval(N, 0, feedbackSequence);

                System.out.println(" ");
                System.out.println("\n off=1.5 evaluation at " + new_eta + ": " + value_150);
                rob_150[indexWF][1] = value_150?1:-1;
                rob_150[indexWF][0] = new_eta;
                indexWF++;
            }

            indexWF=0;
            for(int i = 0; i < 11 ; i++) {
                double new_eta = (eta + i) / 100;
                RobustnessFormula phi_175 = new AtomicRobustnessFormula(
                        ChangeDir(1.75,k),
                        failM,
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        new_eta
                );
                RobustnessFormula phi_fail_175 = new AlwaysRobustnessFormula(
                        phi_175,
                        0,
                        k-1
                );
                Boolean value_175 = new BooleanSemanticsVisitor().eval(phi_fail_175).eval(N, 0, feedbackSequence);

                System.out.println(" ");
                System.out.println("\n off=1.75 evaluation at " + new_eta + ": " + value_175);
                rob_175[indexWF][1]=value_175?1:-1;
                rob_175[indexWF][0]=new_eta;
                indexWF++;
            }

            Util.writeToCSV("./FevalR_125.csv",rob_125);
            Util.writeToCSV("./FevalR_150.csv",rob_150);
            Util.writeToCSV("./FevalR_175.csv",rob_175);

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

    private static void printDataPar(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s1, SystemState s2, int steps, int size) {

        double[][] data = SystemState.sample(rg, F, s1, steps, size);
        double[][] datap = SystemState.sample(rg, F, s2, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>  ", i);
            for (int j = 0; j < data[i].length-1; j++) {
                System.out.printf("%f ", data[i][j]);
                System.out.printf("%f ", datap[i][j]);
            }
            System.out.printf("%f ", data[i][datap[i].length -1]);
            System.out.printf("%f\n", datap[i][datap[i].length -1]);

        }
    }

    private static void printDataPar(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s1, SystemState s2, SystemState s3, int steps, int size) {

        double[][] data1 = SystemState.sample(rg, F, s1, steps, size);
        //double[][] data2 = SystemState.sample(rg, F, s2, steps, size);
        double[][] data3 = SystemState.sample(rg, F, s3, steps, size);
        for (int i = 0; i < data1.length; i++) {
            System.out.printf("%d>  ", i);
            for (int j = 0; j < data1[i].length-1; j++) {
                System.out.printf("%f ", data1[i][j]);
                //System.out.printf("%f ", data2[i][j]);
                System.out.printf("%f ", data3[i][j]);
            }
            System.out.printf("%f ", data1[i][data1[i].length -1]);
            //System.out.printf("%f ", data2[i][data2[i].length -1]);
            System.out.printf("%f\n", data3[i][data3[i].length -1]);

        }
    }

    private static void printLDataPar(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, Perturbation p, SystemState s, int steps, int size) {
        //System.out.println(label);
        double[][] data = SystemState.sample(rg, F, s, steps, size);
        double[][] datap = SystemState.sample(rg, F, p, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>  ", i);
            for (int j = 0; j < data[i].length-1; j++) {
                System.out.printf("%f ", data[i][j]);
                System.out.printf("%f ", datap[i][j]);
            }
            System.out.printf("%f ", data[i][datap[i].length -1]);
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

    private static double[] printMaxData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound){

        /*
        The following instruction creates an evolution sequence consisting in a sequence of <code>steps</code> sample
        sets of cardinality <size>.
        The first sample set contains <code>size</code> copies of configuration <code>s</code>.
        The subsequent sample sets are derived by simulating the dynamics.
        Finally, for each step from 1 to <code>steps</code> and for each variable, the maximal value taken by the
        variable in the elements of the sample set is stored.
         */
        double[][] data_max = SystemState.sample_max(rg, F, new NonePerturbation(), s, steps, size);
        double[] max = new double[F.size()];
        Arrays.fill(max, Double.NEGATIVE_INFINITY);
        for (int i = 0; i < data_max.length; i++) {
            //System.out.printf("%d>   ", i);
            for (int j = 0; j < data_max[i].length -1 ; j++) {
                //System.out.printf("%f   ", data_max[i][j]);
                if (leftbound <= i & i <= rightbound) {
                    if (max[j] < data_max[i][j]) {
                        max[j] = data_max[i][j];
                    }
                }
            }
            //System.out.printf("%f\n", data_max[i][data_max[i].length -1]);
            if (leftbound <= i & i <= rightbound) {
                if (max[data_max[i].length -1] < data_max[i][data_max[i].length -1]) {
                    max[data_max[i].length -1] = data_max[i][data_max[i].length -1];
                }
            }
        }
        System.out.println(" ");
        //System.out.println("Maximal values taken by variables by the unperturbed system:");
        System.out.println(label);
        for(int j=0; j<max.length-1; j++){
            System.out.printf("%f ", max[j]);
        }
        System.out.printf("%f\n", max[max.length-1]);
        System.out.println("");
        System.out.println("");
        return max;
    }

    private static double[] printMaxDataPerturbed(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound, Perturbation perturbation){

        double[] max = new double[F.size()];

        double[][] data_max = SystemState.sample_max(rg, F, perturbation, s, steps, size);
        Arrays.fill(max, Double.NEGATIVE_INFINITY);
        for (int i = 0; i < data_max.length; i++) {
            //System.out.printf("%d>   ", i);
            for (int j = 0; j < data_max[i].length -1 ; j++) {
                //System.out.printf("%f   ", data_max[i][j]);
                if (leftbound <= i & i <= rightbound) {
                    if (max[j] < data_max[i][j]) {
                        max[j] = data_max[i][j];
                    }
                }
            }
            //System.out.printf("%f\n", data_max[i][data_max[i].length -1]);
            if (leftbound <= i & i <= rightbound) {
                if (max[data_max[i].length -1] < data_max[i][data_max[i].length -1]) {
                    max[data_max[i].length -1] = data_max[i][data_max[i].length -1];
                }
            }
        }
        //System.out.println("");
        //System.out.println("Maximal values taken by variables in steps by the perturbed system:");
        System.out.println(label);
        for(int j=0; j<max.length-1; j++){
            System.out.printf("%f ", max[j]);
        }
        System.out.printf("%f\n", max[max.length-1]);
        System.out.println("");
        return max;
    }




    // CONTROLLER OF ROBOT

    public static Controller getController() {

        ControllerRegistry registry = new ControllerRegistry();

        registry.set("SetDir",
                Controller.ifThenElse(
                        DataState.greaterThan(gap,0),
                        Controller.doAction(
                                (rg, ds) -> List.of(new DataStateUpdate(previous_theta,ds.get(theta)),
                                        new DataStateUpdate(theta,
                                        (WPx[(int)ds.get(currentWP)]==ds.get(x))?0:(
                                                (WPx[(int)ds.get(currentWP)]<ds.get(x))?Math.PI:0)+
                                                Math.atan((WPy[(int)ds.get(currentWP)]-ds.get(y))/(WPx[(int)ds.get(currentWP)]-ds.get(x))))
                                        ),
                                registry.reference("Ctrl")
                        ),
                        Controller.ifThenElse(
                                DataState.equalsTo(currentWP,WPx.length-1),
                                Controller.doAction((rg, ds) -> List.of(new DataStateUpdate(timer_V, TIMER)),
                                        registry.reference("Stop")),
                                Controller.doAction((rg, ds) -> List.of(new DataStateUpdate(previous_theta,ds.get(theta)),
                                                new DataStateUpdate(currentWP, ds.get(currentWP)+1),
                                                new DataStateUpdate(theta,
                                                        (WPx[(int)ds.get(currentWP)+1]==ds.get(x))?0:(
                                                                (WPx[(int)ds.get(currentWP)+1]<ds.get(x))?Math.PI:0)+
                                                                Math.atan((WPy[(int)ds.get(currentWP)+1]-ds.get(y))/(WPx[(int)ds.get(currentWP)+1]-ds.get(x))))
                                                ),
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

    /*
    THE FEEDBACK FUNCTION
     */
    private static List<DataStateUpdate> feedbackDirFunction(RandomGenerator randomGenerator, DataState dataState, EvolutionSequence evolutionSequence) {
        int step = dataState.getStep();
        double meanSpeed = evolutionSequence.get(step).mean(ss -> ss.getDataState().get(p_speed));
        double meanTheta = evolutionSequence.get(step).mean(ss -> ss.getDataState().get(theta));
        double meanWP = evolutionSequence.get(step).mean(ss -> ss.getDataState().get(currentWP));
        List<DataStateUpdate> upd = new ArrayList<>();
        if (dataState.get(get_medicine)==1 && dataState.get(p_speed) > MAX_SPEED_WITH_MED-SPEED_DIFFERENCE){
            upd.add(new DataStateUpdate(accel,NEUTRAL));
        }
        if (meanTheta + DIR_DIFFERENCE < dataState.get(theta) || meanTheta - DIR_DIFFERENCE > dataState.get(theta)) {
            upd.add(new DataStateUpdate(previous_theta, dataState.get(theta)));
            if(dataState.get(get_medicine)==0 && dataState.get(p_speed) < MAX_SPEED_WITH_MED){
                upd.add(new DataStateUpdate(theta,
                        (WPx[(int)dataState.get(currentWP)]==dataState.get(x))?0:(
                                (WPx[(int)dataState.get(currentWP)]<dataState.get(x))?Math.PI:0)+
                                Math.atan((WPy[(int)dataState.get(currentWP)]-dataState.get(y))/(WPx[(int)dataState.get(currentWP)]-dataState.get(x))))
                );
            } else {
                upd.add(new DataStateUpdate(theta,
                        (WPx[(int)dataState.get(currentWP)]==dataState.get(x))?0:(
                                (WPx[(int)dataState.get(currentWP)]<dataState.get(x))?Math.PI:0)+
                                Math.atan((WPy[(int)dataState.get(currentWP)]-dataState.get(y))/(WPx[(int)dataState.get(currentWP)]-dataState.get(x)))/2)
                );
            }

        }
        if( dataState.get(s_speed) == 0 & dataState.get(currentWP) < meanWP){
            upd.add(new DataStateUpdate(currentWP, dataState.get(currentWP)+1));
            upd.add(new DataStateUpdate(theta, (WPx[(int)dataState.get(currentWP)+1]==dataState.get(x))?0:(
                    (WPx[(int)dataState.get(currentWP)+1]<dataState.get(x))?Math.PI:0)+
                    Math.atan((WPy[(int)dataState.get(currentWP)+1]-dataState.get(y))/(WPx[(int)dataState.get(currentWP)+1]-dataState.get(x)))));
        }
        if(dataState.get(get_medicine) == 1.0 && dataState.get(flag) == 0.0){
            upd.add(new DataStateUpdate(flag,1.0));
        }
        return upd;
    }


    // ENVIRONMENT

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double new_timer_V = state.get(timer_V) - 1;
        double new_p_speed;
        // the speed is updated according to acceleration
        if (state.get(accel) == NEUTRAL) {
            new_p_speed = Math.max(0.0,state.get(p_speed)-ACCELERATION);
        } else {
            new_p_speed = Math.min(MAX_SPEED, Math.max(0, state.get(p_speed) + state.get(accel)));
        }
        double new_s_speed = new_p_speed; // in the simplified setting, sensed and physical speed coincide
        double newX = state.get(x) + Math.cos(state.get(theta))*new_p_speed; // the position is updated according to
        double newY = state.get(y) + Math.sin(state.get(theta))*new_p_speed; // the physical speed
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
        if (state.get(currentWP)==2 && state.get(get_medicine)==0){
            updates.add(new DataStateUpdate(get_medicine,1)); // medicines are picked up
        }
        if (state.get(currentWP)==8 && state.get(get_medicine)==1){
            updates.add(new DataStateUpdate(get_medicine,0)); // medicines are delivered correctly
        }
        if (Math.abs(state.get(theta)-state.get(previous_theta))>Math.PI/9 && state.get(get_medicine)==1 && new_p_speed > MAX_SPEED_WITH_MED){
            updates.add(new DataStateUpdate(get_medicine,-1));
            updates.add(new DataStateUpdate(fail,1)); // the medicines are dropped because of unsafe movement
        }
        if ((state.get(currentWP)==7 && state.get(get_medicine)!=1) || (state.get(currentWP)==11 && state.get(get_medicine)!=0)){
            updates.add(new DataStateUpdate(fail,1));
        } // medicines are not delivered correctly
        updates.add(new DataStateUpdate(previous_theta,state.get(theta)));
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
        values.put(previous_theta,INIT_THETA);
        values.put(get_medicine,0.0);
        values.put(fail,0.0);
        values.put(flag,0.0);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }


    // PERTURBATIONS

    private static  Perturbation ChangeDir(double off, int k) {
        return new PersistentPerturbation( new AtomicPerturbation(0, (rg,ds)->ds.apply(changeDir(rg,ds,off,k))));
    }

    private static List<DataStateUpdate> changeDir(RandomGenerator rg, DataState state, double off, int k) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double offset = rg.nextDouble() * MAX_THETA_OFFSET - MAX_THETA_OFFSET/2;
        updates.add(new DataStateUpdate(theta, state.get(theta)+offset));
        if (state.getStep() % k == 0){
            updates.add(new DataStateUpdate(p_speed, state.get(p_speed)+rg.nextDouble()*off*ACCELERATION));
        }
        return updates;
    }

}
