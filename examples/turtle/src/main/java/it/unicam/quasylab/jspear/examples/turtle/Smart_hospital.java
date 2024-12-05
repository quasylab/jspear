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

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.distance.AtomicDistanceExpression;
import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.distance.MaxIntervalDistanceExpression;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.perturbation.*;
import it.unicam.quasylab.jspear.feedback.*;
import it.unicam.quasylab.jspear.robtl.AtomicRobustnessFormula;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;
import it.unicam.quasylab.jspear.robtl.ThreeValuedSemanticsVisitor;
import it.unicam.quasylab.jspear.robtl.TruthValues;
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
    public final static double MAX_THETA_OFFSET = 0.1;
    public final static double INIT_X = 15.0;
    public final static double INIT_Y = 6.0;
    public final static double INIT_THETA = Math.PI/2;
    public final static double FINAL_X = 15.0;
    public final static double FINAL_Y = 6.0;
    public final static double[] WPx = {13,13,13,6,6,2,6,6,2,6,6,FINAL_X};
    public final static double[] WPy = {6,1,6,6,2,2,2,7,7,7,6,FINAL_Y};
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
    private static final int previous_theta = 10;
    private static final int get_medicine = 11; // 0 if the robot is not transporting medicines. 1 otherwise.
    private static final int fail = 12; // 0 if the robot delivered correctly the medicines. 1 otherwise.

    private static final int NUMBER_OF_VARIABLES = 13;
    private static final double DIR_DIFFERENCE = 0.001;
    private static final double SPEED_DIFFERENCE = 0.05;


    public static void main(String[] args) throws IOException {
        try {

            RandomGenerator rand = new DefaultRandomGenerator();

            /*

            INITIAL CONFIGURATION

            In order to perform simulations/analysis/model checking for a particular system, we need to create its
            initial configuration, which is an instance of <code>ControlledSystem>/code>

            */


            /*
            One of the elements of a system configuration is the "controller", i.e. an instance of <code>Controller</code>.
            Here, the controller named <code>robot</code> is returned by static method <code>getController</code>.
             */

            Controller robot = getController();

            /*
            Another element of a system configuration is the "data state", i.e. an instance of <code>DataState</code>,
            which models the state of the data.
            Instances of <code>DataState</code> contains values for variables representing the quantities of the
            system.
            The initial state <code>state</code> is constructed by exploiting the static method
            <code>getInitialState</code>, which will be defined later and assigns the initial value to all
            variables defined above.
             */

            DataState state = getInitialState();

            /*
            We define the <code>ControlledSystem</code> <code>system</code>, which will be the starting configuration from
            which the evolution sequence will be constructed.
            This configuration consists of 3 elements:
            - the controller <code>robot</code> defined above,
            - a random function over data states, which implements interface <code>DataStateFunction</code> and maps a
            random generator <code>rg</code> and a data state <code>ds</code> to a new data state,
            - the data state <code>state</state> defined above.
             */

            ControlledSystem system = new ControlledSystem(robot, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);

            int sizeNominalSequence = 100;

            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, sizeNominalSequence);

            /*
            Below we define a feedback, namely an element of <code>Feedback</code>.
            In this case, <code>feedbackSpeedAndDir</code> is a <code>PersistentFeedback</code>, namely at each
            evolution step its body is applied, where the body is the <code>AtomicFeedback</code>
            <code>feedbackSpeedAndDir</code>, which is derived by the evolution sequence <code>sequence</code> and applies
            at each step the <code>FeedbackFunction</code> returned by static method <code>feedbackSpeedAndDirFunction</code>.
             */
            Feedback feedbackDir = new PersistentFeedback(new AtomicFeedback(0, sequence, Smart_hospital::feedbackDirFunction));

            /*
            Below we define a <code>FeedbackSystem</code> named <code>feedbackSystem</code>, which, essentially,
            is a version of <code>system</code> equipped by feedback <code>feedbackSpeedAndDir</code>.
             */
            FeedbackSystem feedbackSystem = new FeedbackSystem(robot, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state, feedbackDir);

            EvolutionSequence feedbackSequence = new EvolutionSequence(rand, rg -> feedbackSystem, sizeNominalSequence);

            Perturbation perturbation = new PersistentPerturbation(new AtomicPerturbation(0, (rg,ds)-> ds.apply(changeDir(rg,ds,ACCELERATION/2))));

            PerturbedSystem perturbedSystem = new PerturbedSystem(system, perturbation);
            PerturbedSystem perturbedFeedbackSystem = new PerturbedSystem(feedbackSystem, perturbation);


            /*
            USING THE SIMULATOR
             */

            int N = 300;

            ArrayList<String> L = new ArrayList<>();
            L.add("x");
            L.add("y");
            L.add("theta");
            L.add("speed");
            L.add("waypoint");
            L.add("diff_angle");
            L.add("got_med");
            L.add("failure");

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(x));
            F.add(ds->ds.get(y));
            F.add(ds->ds.get(theta));
            F.add(ds->ds.get(p_speed));
            F.add(ds->(int)ds.get(currentWP));
            F.add(ds->ds.get(previous_theta));
            F.add(ds->(int)ds.get(get_medicine));
            F.add(ds->(int)ds.get(fail));

            printDataPar(rand,L,F,system,perturbedSystem,perturbedFeedbackSystem,N,sizeNominalSequence);


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

            double[][] pfdata = SystemState.sample(rand, F, perturbation, perturbedFeedbackSystem, N, sizeNominalSequence);
            for (int i = 0; i<N; i++){
                position_perturbed_feedback_system[i][0] = pfdata[i][0];
                position_perturbed_feedback_system[i][1] = pfdata[i][1];
                get_med_perturbed_feedback_system[i][0] = pfdata[i][6];
                fail_perturbed_feedback_system[i][0] = pfdata[i][7];
            }
            Util.writeToCSV("./xy_feedback.csv",position_perturbed_feedback_system);
            Util.writeToCSV("./get_med_feedback.csv",get_med_perturbed_feedback_system);
            Util.writeToCSV("./fail_feedback.csv",fail_perturbed_feedback_system);

            /*

            ESTIMATING BEHAVIORAL DISTANCES BETWEEN EVOLUTION SEQUENCES


            Now we generate again three sequences:
            1. a nominal sequence
            2. a perturbed sequence for the system without feedback
            3. a perturbed sequence for the system equipped with feedback.
            Then, we quantify the differences between the evolutions sequences #1 and #2, which corresponds
            to quantifying the behavioural distance between the nominal and the perturbed system without feedback,
            and between the evolutions sequences #1 and #3, which corresponds to quantifying the behavioural distance
            between the nominal and the perturbed system equipped with feedback.
            The differences are expressed with respect to the points in the plane reached by the systems.
             */

            /*
            The following instruction allows us to create the evolution sequence <code>perturbedSequence</code>, which is
            obtained from the evolution sequence <code>sequence</code> by applying a perturbation, where:
            - as above, the perturbation is  <code>perturbation</code>
            - the perturbation is applied at step 0
            - the sample sets of configurations in <code>perturbedSequence</code> have a cardinality which corresponds to that
            of <code>sequence</code> multiplied by <code>scale>/code>
            Moreover, we create also the evolution sequence <code>perturbedFeedbackSequence</code>, which is
            obtained from the evolution sequence <code>feedbackSequence</code> by applying <code>perturbation</code>
            */

            int scale=500;
            EvolutionSequence perturbedFeedbackSequence = feedbackSequence.apply(perturbation,0,scale);

            /*
            The following lines of code first defines an atomic distance between evolution sequences, named
            <code>distP2P</code>. Then, these distances are evaluated, time-point by time-point, over
            evolution sequence <code>sequence</code> and its perturbed version <code>perturbedSequence</code> defined above,
            and over <code>sequence</code> and its perturbed version with feedback <code>perturbedFeedbackSequence</code>.
            Finally, the time-point to time-point values of the distances are stored in .csv files and printed out.
            Technically, <code>distP2P</code> is an atomic distance in the sense that it is an instance of
            class <code>AtomicDistanceExpression</code>, which consists in a data state expression,
            which maps a data state to a number, or rank, and a binary operator. As already discussed, in this case,
            given two configurations, the data state expression allow us to get the normalised distance from the origin,
            which is a value in [0,1], from both configuration, and the binary operator gives us their difference, which,
            intuitively, is the difference between the two configurations with respect to their position.
            This distance will be lifted to two sample sets of configurations, those obtained from the compared
            sequences at the same step.
             */

            AtomicDistanceExpression distSpeed = new AtomicDistanceExpression(ds->(ds.get(p_speed)/MAX_SPEED), (v1, v2) -> Math.abs(v2-v1));

            int leftBound = 0;
            int rightBound = 300;

            double[][] direct_evaluation_atomic_distSpeed = new double[rightBound-leftBound][1];

            for (int i = 0; i<(rightBound-leftBound); i++){
                direct_evaluation_atomic_distSpeed[i][0] = distSpeed.compute(i+leftBound, sequence, perturbedFeedbackSequence);
            }

            Util.writeToCSV("./atomic_speed_nf.csv",direct_evaluation_atomic_distSpeed);


            AtomicDistanceExpression distTheta = new AtomicDistanceExpression(ds->(ds.get(theta)/(Math.PI*2)), (v1, v2) -> Math.abs(v2-v1));
            double[][] direct_evaluation_atomic_distTheta = new double[rightBound-leftBound][1];

            for (int i = 0; i<(rightBound-leftBound); i++){
                direct_evaluation_atomic_distTheta[i][0] = distTheta.compute(i+leftBound, sequence, perturbedFeedbackSequence);
            }

            Util.writeToCSV("./atomic_theta_nf.csv",direct_evaluation_atomic_distTheta);







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
        //System.out.println("Maximal values taken by variables by the non perturbed system:");
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
    The following feedback works as follows.
    The present status of the PT is compared with the status that the DT reached at the same instant. Then:
    1. If the sensed speed of the PT is much higher than the speed of the DT, then braking is activated.
    2. If the sensed speed of the PT is much lower than the speed of the DT, then acceleration is activated.
    3. If the waypoint of the PT is not the waypoint of the DT, then the waypoint of the PT is corrected. Contextually,
    also the direction of the PT is corrected.
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
        return upd;
    }


    // ENVIRONMENT EVOLUTION

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double new_timer_V = state.get(timer_V) - 1; // timer is simply decremented
        double new_p_speed;
        // the speed is updated according to acceleration
        if (state.get(accel) == NEUTRAL) {
            new_p_speed = Math.max(0.0,state.get(p_speed)-ACCELERATION);
        } else {
            new_p_speed = Math.min(MAX_SPEED, Math.max(0, state.get(p_speed) + state.get(accel)));
        }
        double new_s_speed = new_p_speed; // the sensed speed corresponds to the physical one in case of no perturbation
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
            updates.add(new DataStateUpdate(get_medicine,1));
        }
        if (state.get(currentWP)==8 && state.get(get_medicine)==1){
            updates.add(new DataStateUpdate(get_medicine,0));
        }
        if (Math.abs(state.get(theta)-state.get(previous_theta))>Math.PI/9 && state.get(get_medicine)==1 && new_p_speed > MAX_SPEED_WITH_MED){
            updates.add(new DataStateUpdate(get_medicine,-1));
            updates.add(new DataStateUpdate(fail,1));
        }
        if ((state.get(currentWP)==7 && state.get(get_medicine)!=1) || (state.get(currentWP)==11 && state.get(get_medicine)!=0)){
            updates.add(new DataStateUpdate(fail,1));
        }
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
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }


    // PERTURBATIONS

    private static  Perturbation getIteratedChangeDir(double off) {
        return new AfterPerturbation(1, new IterativePerturbation(10, new AtomicPerturbation(10, (rg,ds)->ds.apply(changeDir(rg,ds,off)))));
    }

    private static List<DataStateUpdate> changeDir(RandomGenerator rg, DataState state, double off) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double offset = rg.nextDouble() * MAX_THETA_OFFSET - MAX_THETA_OFFSET/2;
        updates.add(new DataStateUpdate(theta, state.get(theta)+offset));
        if (state.getStep() % 5 == 0){
            updates.add(new DataStateUpdate(p_speed, state.get(p_speed)+rg.nextDouble()*off));
        }
        return updates;
    }

}
