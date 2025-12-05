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
import it.unicam.quasylab.jspear.distance.MaxDistanceExpression;
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

public class Industrial_plant {

    public final static String[] VARIABLES =
            new String[]{"p_speed", "s_speed", "p_distance", "accel", "timer_V", "braking_distance", "gap"
            };

    public final static double ACCELERATION = 0.05; // acceleration, m/s^2
    public final static double BRAKE = 0.40; // negative acceleration, m/s^2
    public final static double NEUTRAL = 0.0; // neutral acceleration
    public final static int TIMER = 1; // frequency of working cycle by controller
    public final static double INIT_SPEED = 0.0; // initial speed of the robot
    public final static double MAX_SPEED = 3.0; // maximal speed
    public final static double MAX_SPEED_OFFSET = 0.15; // maximal perturbation on the speed
    public final static double INIT_X = 0.0; // initial x position
    public final static double INIT_Y = 0.0; // initial y position
    public final static double INIT_THETA = Math.PI/2; // initial steering angle
    public final static double FINAL_X = 35.0; // last waypoint, x coordinate
    public final static double FINAL_Y = 30.0; // last waypoint, y coordinate
    public final static double[] WPx = {1,13,7,23,20,32,FINAL_X}; // list of x coordinate of waypoints
    public final static double[] WPy = {3,3,7,14,31,40,FINAL_Y}; // list of y coordinate of waypoints
    public final static double INIT_DISTANCE = Math.sqrt(Math.pow((WPx[0]-INIT_X),2) + Math.pow((WPy[0]-INIT_Y),2));



    private static final int x = 0; // current position, x coordinate
    private static final int y = 1; // current position, y coordinate
    private static final int theta = 2; // current direction
    private static final int p_speed = 3; // physical speed (m/s)
    private static final int s_speed = 4; // sensed speed (m/s)
    private static final int p_distance = 5; // auxiliary variable, physical distance from the current waypoint
    private static final int accel = 6; // acceleration
    private static final int timer_V = 7; // timer
    private static final int gap = 8; // auxiliary variable, difference between p_distance and the space required to stop when braking
    private static final int currentWP = 9; // current waypoint

    private static final int NUMBER_OF_VARIABLES = 10;
    private static final double SPEED_DIFFERENCE = 0.0001;


    public static void main(String[] args) throws IOException {
        try {

            RandomGenerator rand = new DefaultRandomGenerator();

            /*

            INITIAL CONFIGURATION

            In order to perform simulations/analysis/model checking for a particular system, we need to create its
            initial configuration, which is an instance of <code>ControlledSystem</code>

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
            <code>getInitialState</code>, which will be defined later and assigns the initial value to the 10
            variables defined above.
             */
            DataState state = getInitialState();


            /*
            We define the <code>ControlledSystem</code> <code>system</code>, which will be the starting configuration from
            which the evolution sequence will be constructed.
            This configuration consists of 3 elements:
            - the controller <code>robot</code> defined above, which model the cyber component of the system;
            - a random function over data states, which implements interface <code>DataStateFunction</code> and maps a
            random generator <code>rg</code> and a data state <code>ds</code> to a new data state. This function models
            the physical component of the system;
            - the data state <code>state</state> defined above.
             */

            ControlledSystem system = new ControlledSystem(robot, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);



            /*
            Below we create an evolution sequence.
            Variable <code>sizeNominalSequence</code> gives the number of runs that are used to simulate the evolution
            sequence.
            More in detail, an evolution sequence, modelled by class <code>EvolutionSequence</code>, is a sequence of
            sample sets of system configurations, where configurations are modelled by class <code>ControlledSystem</code>
            and sample sets by class <code>SampleSet</code>.
            In this context, <code>sizeNominalSequence</code> is the cardinality of those sample sets.
            The evolution sequence is created as a sequence containing only one sample set of configurations, which
            consists in <code>sizeNominalSequence</code> of configuration <code>system</code>.
            The subsequent sample sets will be generated "on demand".

            */
            int sizeNominalSequence = 10;

            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, sizeNominalSequence);

            /*
            Below we define a feedback, namely an instance of <code>Feedback</code>.
            In this case, <code>feedbackSpeedAndDir</code> is a <code>PersistentFeedback</code>, namely at each
            evolution step its body is applied, where the body is an atomic feedback, namely an instance of
            <code>AtomicFeedback</code>. An atomic feedback consists in a delay, which is 0 in this case, an
            evolution sequence, which is <code>sequence</code> in this case, and a feedback function, namely an instance
            of <code>FeedbackFunction</code> that is returned by static method <code>feedbackSpeedAndDir</code> in
            this case.
            Essentially, <code>sequence</code> will play the role of the DT.
             */
            Feedback feedbackSpeedAndDir = new PersistentFeedback(new AtomicFeedback(0, sequence, Industrial_plant::feedbackSpeedAndDirFunction));

            /*
            Below we define a <code>FeedbackSystem</code> named <code>feedbackSystem</code>, which, essentially,
            is a version of <code>system</code> equipped with feedback <code>feedbackSpeedAndDir</code>.
             */
            FeedbackSystem feedbackSystem = new FeedbackSystem(robot, (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state, feedbackSpeedAndDir);

            /*
            Below we define an evolution sequence starting from <code>feedbackSystem</code>.
            This evolution sequence, named <code>feedbackSequence</code> has the same size of <code>sequence</code>.
             */
            EvolutionSequence feedbackSequence = new EvolutionSequence(rand, rg -> feedbackSystem, sizeNominalSequence);


            /*
            Below we define a perturbation, namely an instance of <code>Perturbation</code>.
            In this case, <code>perturbation</code> is a <code>PersistentPerturbation</code>, namely at each
            evolution step its body is applied, where the body is the <code>AtomicPerturbation</code>
            which perturbs the data states by applying the </code>DataStateFunction</code> returned by
            static method <code>slowerPerturbation</code>.
             */
            Perturbation perturbation = new PersistentPerturbation(new AtomicPerturbation(0, Industrial_plant::slowerPerturbation));

            /*
            The systems <code>perturbedSystem</code> and <code>perturbedFeedbackSystem</code> defined below
            are the perturbed versions of <code>system</code> and <code>feedbackSystem</code>, respectively.
              */
            PerturbedSystem perturbedSystem = new PerturbedSystem(system, perturbation);
            PerturbedSystem perturbedFeedbackSystem = new PerturbedSystem(feedbackSystem, perturbation);

            /*
            USING THE SIMULATOR
            */

            ArrayList<String> L = new ArrayList<>();
            L.add("      x   ");
            L.add("   y   ");
            L.add(" theta");
            L.add(" p_speed");
            L.add("s_speed");
            L.add("distance");
            L.add("gap ");
            L.add(" waypoint");
            L.add("    x   ");
            L.add(" y   ");
            L.add("  theta");
            L.add("  p_speed");
            L.add("  s_speed");
            L.add("  distance");
            L.add("  gap");
            L.add("  waypoint ");
            L.add("  x  ");
            L.add("  y  ");
            L.add("  theta");
            L.add("  p_speed");
            L.add("  s_speed");
            L.add(" distance");
            L.add(" gap ");
            L.add(" waypoint ");

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(x));
            F.add(ds->ds.get(y));
            F.add(ds->ds.get(theta));
            F.add(ds->ds.get(p_speed));
            F.add(ds->ds.get(s_speed));
            F.add(ds->ds.get(p_distance));
            F.add(ds->ds.get(gap));
            F.add(ds->ds.get(currentWP));

            /*
            We start with generating three evolution sequences of length <code>N</code> of sample sets of cardinality
            <code>sizeNominalSequence</code> of configurations, with the first sample set consisting in
            <code>sizeNominalSequence</code> copies of <code>system</code>, <code>perturbedSystem</code> or
            <code>perturbedFeedbackSystem</code>, respectively.
            For each evolution sequence and step in [0,N-1], and for each variable, we print out
            the average value that the variable assumes in the <code>sizeNominalSequence</code> configurations in the
            sample set obtained at that step.
            The simulator, which is offered by method <code>sample</code> of <code>SystemState</code>,
            is called by method <code>printDataPar</code>.
            */

            int N = 300;
            System.out.println("Simulation of " + N + " steps of a nominal, a perturbed and a perturbed with feedback system");
            System.out.println("");
            System.out.println(L);
            printDataPar(rand,L,F,system,perturbedSystem,perturbedFeedbackSystem,N,sizeNominalSequence);
            System.out.println("");
            System.out.println("");


            /*
            Below we repeat the simulation, but instead of printing out the results we store them in a .csv file
            the value obtained for <code>x</code> and <code>y</code>.
            */

            double[][] plot_system = new double[N][2];
            double[][] plot_perturbed_system = new double[100][2];
            double[][] plot_perturbed_feedback_system = new double[N][2];


            double[][] data = SystemState.sample(rand, F, system, N, sizeNominalSequence);
            for (int i = 0; i<N; i++){
                plot_system[i][0] = data[i][0];
                plot_system[i][1] = data[i][1];
            }
            Util.writeToCSV("./Fnew_plotxy.csv",plot_system);


            double[][] pdata = SystemState.sample(rand, F, perturbation, system, 100, 5);
            for (int i = 0; i<100; i++){
                plot_perturbed_system[i][0] = pdata[i][0];
                plot_perturbed_system[i][1] = pdata[i][1];
            }
            Util.writeToCSV("./Fnew_pplotxy.csv",plot_perturbed_system);

            double[][] pfdata = SystemState.sample(rand, F, perturbation, perturbedFeedbackSystem, N, sizeNominalSequence);
            for (int i = 0; i<N; i++){
                plot_perturbed_feedback_system[i][0] = pfdata[i][0];
                plot_perturbed_feedback_system[i][1] = pfdata[i][1];
            }
            Util.writeToCSV("./Fnew_pfplotxy.csv",plot_perturbed_feedback_system);


            /*

            ESTIMATING BEHAVIORAL DISTANCES BETWEEN EVOLUTION SEQUENCES


            Now we generate again three sequences:
            1. a nominal sequence
            2. a perturbed sequence for the system without feedback
            3. a perturbed sequence for the system equipped with feedback.
            Then, we quantify the step-by-step differences between the evolutions sequences #1 and #2, which corresponds
            to quantifying the behavioural distance between the nominal and the perturbed system without feedback,
            and between the evolutions sequences #1 and #3, which corresponds to quantifying the behavioural distance
            between the nominal and the perturbed system equipped with feedback.
            The differences are expressed with respect to the distance of the robot from the next waypoint.
             */
/*

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

            int scale=5;
            EvolutionSequence perturbedSequence = sequence.apply(perturbation,0,scale);
            EvolutionSequence perturbedFeedbackSequence = feedbackSequence.apply(perturbation,0,scale);



            /*
            In order to quantify the difference between two evolution sequences, we need to define a notion of distance
            between two configurations.


            We start with simulating the nominal and the perturbed system and with computing the maximal Euclidean distance
            between the position of the robot and the current waypoint that can be observed in all configurations generated
            in those simulations.
            The value obtained will be exploited for normalisation purposes.
            */






            int size = 5;
            System.out.println("");
            System.out.println("Simulation of nominal system - Data maximal values:");
            double dataMax = printMaxDataWP(rand, L, F, system, N, size, 0,2*N);
            System.out.println("");
            System.out.println("Simulation of perturbed system - Data maximal values:");
            System.out.println("");
            double dataMax_p = printMaxDataWPPerturbed(rand, L, F, system, N, size, 0, 2*N, perturbation);
            double normalisationF = Math.max(dataMax,dataMax_p);



            /*
            The following lines of code first defines an atomic distance between evolution sequences, named
            <code>distP2P</code>. Then, this distance is evaluated, step-by-step, over
            evolution sequence <code>sequence</code> and its perturbed version <code>perturbedSequence</code> defined above,
            and over <code>sequence</code> and its perturbed version with feedback <code>perturbedFeedbackSequence</code>.
            Finally, the step-by-step values of the distances are stored in .csv files and printed out.

            Technically, <code>distP2P</code> is an atomic distance in the sense that it is an instance of
            class <code>AtomicDistanceExpression</code>, which consists in a data state expression,
            which maps a data state to a number, or rank, and a binary operator.

            In our case, the data state expression allow us to get the normalised distance between the robot and the
            current waypoint, and the binary operator gives us their difference, which, intuitively, tells us how
            closer to the current waypoint is one robot with respect to the other.
            This distance will be lifted to two sample sets of configurations, those obtained from the compared
            sequences at the same step. This lifting is done by method <code>compute</code> of <code>DistanceExpression</code>.
            */


            DistanceExpression distP2P =  new AtomicDistanceExpression(ds->(Math.sqrt(Math.pow((ds.get(x)-WPx[(int)ds.get(currentWP)]),2)+Math.pow((ds.get(y)-WPy[(int)ds.get(currentWP)]),2)))/normalisationF, (v1, v2) -> Math.abs(v2-v1));

            int leftBound = 0;
            int rightBound = 200;

            double[][] direct_evaluation_atomic_distP2P = new double[rightBound-leftBound][1];

            for (int i = 0; i<(rightBound-leftBound); i++){
                direct_evaluation_atomic_distP2P[i][0] = distP2P.compute(i+leftBound, sequence, perturbedSequence);
            }

            Util.writeToCSV("./Fatomic_P2P.csv",direct_evaluation_atomic_distP2P);

            System.out.println("ciao");

            for(int i=0 ; i < direct_evaluation_atomic_distP2P.length; i++){
                System.out.println(direct_evaluation_atomic_distP2P[i][0]);

            }

            for (int i = 0; i<(rightBound-leftBound); i++){
                direct_evaluation_atomic_distP2P[i][0] = distP2P.compute(i+leftBound, sequence, perturbedFeedbackSequence);
            }

            Util.writeToCSV("./Fatomic_FP2P.csv",direct_evaluation_atomic_distP2P);

            System.out.println("ciao");

            for(int i=0 ; i < direct_evaluation_atomic_distP2P.length; i++){
                System.out.println(direct_evaluation_atomic_distP2P[i][0]);

            }



            /*
            USING THE MODEL CHECKER

            The distance <code>intP2PMax</code> defined below returns the maximal value given by the evaluation of
            <code>distP2P</code> in an interval.
            We define a robustness formula, in particular an atomic formula, namely an instance of
            <code>AtomicRobustnessFormula</code>.
            This formula will be evaluated on the evolution sequence <code>sequence</code> and expresses that the
            distance, expressed by expression distance <code>intP2PMax</code> between that evolution
            sequence and the evolution sequence obtained from it by applying the perturbation <code>perturbation</code>
            defined above, is below a given threshold.
            For several thresholds, we print out and store in a .csv file the result.


             */
            int leftRBound=0;
            int rightRBound=200;

            /*

             */

            DistanceExpression intP2PMax = new MaxIntervalDistanceExpression(
                    distP2P,
                    leftRBound,
                    rightRBound
            );

            double[][] robEvaluationsWF = new double[20][2];
            RobustnessFormula robustFWF;
            int indexWF=0;
            double thresholdBWF = 30;
            for(int i = 0; i < 20 ; i=i+1){
                double thresholdWF = thresholdBWF + 2*i;
                thresholdWF = thresholdWF / 100;
                robustFWF = new AtomicRobustnessFormula(
                        perturbation,
                        intP2PMax,
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        thresholdWF);
                TruthValues value = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(robustFWF).eval(5, 0, sequence);
                System.out.println(" ");
                System.out.println("\n robustF evaluation at " + thresholdWF + ": " + value);
                robEvaluationsWF[indexWF][1]=value.valueOf();
                robEvaluationsWF[indexWF][0]=thresholdWF;
                indexWF++;
            }

            Util.writeToCSV("./FevalR.csv",robEvaluationsWF);


            /*
            Below we re-employ the model checker for <code>feedbackSequence</code>.
             */

            double[][] robEvaluations = new double[20][2];
            RobustnessFormula robustF;
            int index=0;
            double thresholdB = 1;
            for(int i = 0; i < 20 ; i++){
                double threshold = thresholdB + i;
                threshold = threshold / 100;
                robustF = new AtomicRobustnessFormula(
                        perturbation,
                        intP2PMax,
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        threshold);
                TruthValues value = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(robustF).eval(5, 0, feedbackSequence);
                System.out.println(" ");
                System.out.println("\n robustF evaluation at " + threshold + ": " + value);
                robEvaluations[index][1]=value.valueOf();
                robEvaluations[index][0]=threshold;
                index++;
            }

            Util.writeToCSV("./FevalRF.csv",robEvaluations);



        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }


    private static List<DataStateUpdate> feedbackSpeedFunction(RandomGenerator randomGenerator, DataState dataState, EvolutionSequence evolutionSequence) {
        int step = dataState.getStep();
        double meanSpeed = evolutionSequence.get(step).mean(ss -> ss.getDataState().get(p_speed));
        if (meanSpeed  + SPEED_DIFFERENCE < dataState.get(p_speed)) {
            return List.of(new DataStateUpdate(accel, -BRAKE));
        }
        if (meanSpeed - SPEED_DIFFERENCE > dataState.get(p_speed)) {
            return List.of(new DataStateUpdate(accel, ACCELERATION));
        }
        return List.of();
    }

    /*
    The following feedback works as follows.
    The present status of the PT is compared with the status that the DT reached at the same instant. Then:
    1. If the sensed speed of the PT is much higher than the speed of the DT, then braking is activated.
    2. If the sensed speed of the PT is much lower than the speed of the DT, then acceleration is activated.
    3. If the waypoint of the PT is not the waypoint of the DT, then the waypoint of the PT is corrected. Contextually,
    also the direction of the PT is corrected.
     */
    private static List<DataStateUpdate> feedbackSpeedAndDirFunction(RandomGenerator randomGenerator, DataState dataState, EvolutionSequence evolutionSequence) {
        int step = dataState.getStep();
        double meanSpeed = evolutionSequence.get(step).mean(ss -> ss.getDataState().get(p_speed));
        double meanWP = evolutionSequence.get(step).mean(ss -> ss.getDataState().get(currentWP));
        List<DataStateUpdate> upd = new ArrayList<>();
        if (meanSpeed  + SPEED_DIFFERENCE < dataState.get(s_speed)) {
            upd.add(new DataStateUpdate(accel, -BRAKE));
        }
        if (meanSpeed - SPEED_DIFFERENCE > dataState.get(s_speed)) {
            upd.add(new DataStateUpdate(accel, ACCELERATION));
        }
        if( dataState.get(s_speed) == 0 & dataState.get(currentWP) < meanWP){
            upd.add(new DataStateUpdate(currentWP, dataState.get(currentWP)+1));
            upd.add(new DataStateUpdate(theta, (WPx[(int)dataState.get(currentWP)+1]==dataState.get(x))?0:((WPx[(int)dataState.get(currentWP)+1]<dataState.get(x))?Math.PI:0)+Math.atan((WPy[(int)dataState.get(currentWP)+1]-dataState.get(y))/(WPx[(int)dataState.get(currentWP)+1]-dataState.get(x)))));
        }
        return upd;
    }











    private static void printDataPar(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s1, SystemState s2, SystemState s3, int steps, int size) {

        double[][] data1 = SystemState.sample(rg, F, s1, steps, size);
        double[][] data2 = SystemState.sample(rg, F, s2, steps, size);
        double[][] data3 = SystemState.sample(rg, F, s3, steps, size);
        for (int i = 0; i < data1.length; i++) {
            System.out.printf("%d>  ", i);
            for (int j = 0; j < data1[i].length-1; j++) {
                System.out.printf("%f ", data1[i][j]);
                System.out.printf("%f ", data2[i][j]);
                System.out.printf("%f ", data3[i][j]);
            }
            System.out.printf("%f ", data1[i][data1[i].length -1]);
            System.out.printf("%f ", data2[i][data2[i].length -1]);
            System.out.printf("%f\n", data3[i][data3[i].length -1]);

        }
    }







    private static double printMaxDataWP(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound){
        double[][] data_max = SystemState.sample_max(rg, F, new NonePerturbation(), s, steps, size);
        double[] data_euc = new double[steps];
        double max = Double.NEGATIVE_INFINITY;
        for(int i = 0; i<data_euc.length;i++){
            data_euc[i] = Math.sqrt( (Math.pow((data_max[i][0] - WPx[(int)data_max[i][7]]) , 2)) + (Math.pow((data_max[i][1] - WPy[(int)data_max[i][7]]),2)) );
            if(data_euc[i] > max){
                max = data_euc[i];
            }
        }
        return max;
    }

    private static double printMaxDataWPPerturbed(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound, Perturbation p){
        double[][] data_max = SystemState.sample_max(rg, F, p, s, steps, size);
        double[] data_euc = new double[steps];
        double max = Double.NEGATIVE_INFINITY;
        for(int i = 0; i<data_euc.length;i++){
            data_euc[i] = Math.sqrt( (Math.pow((data_max[i][0] - WPx[(int)data_max[i][7]]) , 2)) + (Math.pow((data_max[i][1] - WPy[(int)data_max[i][7]]),2)) );
            if(data_euc[i] > max){
                max = data_euc[i];
            }
        }
        return max;
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

        double new_s_speed = new_p_speed; // the sensed speed corresponds to the physical one in case of no perturbation
        double newX = state.get(x) + Math.cos(state.get(theta))*new_p_speed; // the position is updated according to
        double newY = state.get(y) + Math.sin(state.get(theta))*new_p_speed; // the physical speed

        double new_p_distance = Math.sqrt(Math.pow(WPx[(int)state.get(currentWP)]-newX,2) + Math.pow(WPy[(int)state.get(currentWP)]-newY,2));

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



    private static DataState slowerPerturbation(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        //double offset = MAX_SPEED_OFFSET;
        double offset = rg.nextDouble() * MAX_SPEED_OFFSET;
        double fake_speed = Math.max(0, state.get(p_speed) - offset);
        double fake_braking_distance = (Math.pow(fake_speed,2) + (ACCELERATION + BRAKE) * (ACCELERATION * Math.pow(TIMER,2) +
                2 * fake_speed * TIMER)) / (2 * BRAKE);
        double fake_gap = state.get(p_distance) - fake_braking_distance;
        updates.add(new DataStateUpdate(s_speed, fake_speed));
        updates.add(new DataStateUpdate(gap, fake_gap));
        return state.apply(updates);
    }

}