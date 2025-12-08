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

package it.unicam.quasylab.jspear.examples.Lotka;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.NilController;
import it.unicam.quasylab.jspear.distance.*;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.perturbation.AtomicPerturbation;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.perturbation.*;
import it.unicam.quasylab.jspear.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.*;

public class Main {

    /*
    LOTKA REACTIONS

    The well known set of Lotka reactions consist in the following two autocatalitic reactions
     X + Y1  ---c1---> 2Y1
     Y1 + Y2 ---c2---> 2Y2
    plus the isomerization
     Y2      ---c3---> Z
    where it is assumed that the amount of X is insignificantly depleted by the first reaction.
    The behaviour of Y1 and Y2 is oscillatory.

    */




    /*

    ARRAYS REPRESENTING REACTANTS/PRODUCTS OF REACTIONS

    Technically, for each of the 3 reactions, we define 2 arrays.
    Each of these arrays has 4 positions, which are associated to the four species.
    In particular, the 4 positions are for: X, Y1, Y2, Z.
    Then, the two arrays for each reaction ri, with i=1,2,3 are:
    - ri_input: position j is 1 if the variable corresponding to position j is a reactant of the reaction
    - ro_output: position j is 1 if the variable corresponding to j is a product of the reaction
    The arrays are defined below.
    For each reaction ri we consider also the constant ri_k carrying the value of ci.
    */


    /*
     Reaction r1 is X + Y1 ---c1---> 2Y1
    */
    public static final int[] r1_input = {1,1,0,0};
    public static final int[] r1_output = {1,2,0,0};
    public static final double r1_k = 0.01;

    /*
    Reaction r2 is Y1 + Y2 ---c2---> 2Y2.
    */
    public static final int[] r2_input =  {0,1,1,0};
    public static final int[] r2_output = {0,0,2,0};
    public static final double r2_k = 0.01;

    /*
    Reaction r3 is Y2 ---c3---> Z.
    */
    public static final int[] r3_input =  {0,0,1,0};
    public static final int[] r3_output = {0,0,0,1};
    public static final double r3_k = 10.0;


    public static final int[][] r_input = {r1_input,r2_input,r3_input};
    public static final double[] r_k = {r1_k,r2_k,r3_k};


    /*

    VARIABLES MODELING THE STATUS OF THE SYSTEM

    Below a list of 4 variables, the idea being that the value of these 4 variables gives a data state, namely an
    instance of class <code>DataState</code> representing the status of all quantities of the system.
    Each variable is associated with an index, from 0 to 29.
    */
    public static final int X = 0; // amount of molecules of X
    public static final int Y1 = 1; // amount of molecules of Y1.
    public static final int Y2 = 2; // amount of molecules of Y2.
    public static final int Z = 3; // amount of molecules of Z.

    private static final int NUMBER_OF_VARIABLES = 4;



    // MAIN PROGRAM
    public static void main(String[] args) throws IOException {
        try {
            /*

            INITIAL CONFIGURATION

            In order to perform simulations/analysis/model checking for a particular system, we need to create its
            initial configuration, which is an instance of <code>TimedSystem>/code>

            */

            /*
            One of the elements of a system configuration is the "controller", i.e. an instance of <code>Controller</code>.
            In this example we do not need controllers, therefore we use a controller that does nothing, i.e. an instance
            of <code>NilController</code>.
            In other case studies, controllers may be used to control the activity of a system.
            For instance, a scheduling of a therapy may be modelled by a controller.
            */
            Controller controller = new NilController();

            /*
            Another element of a system configuration is the "data state", i.e. an instance of <code>DataState</code>,
            which models the state of the data.
            Instances of <code>DataState</code> contains values for variables representing the quantities of the
            system and four values allowing us to model the evolution of time: gran, Tstep, Treal, Tdelta.
            The initial state <code>state</code> is constructed by exploiting the static method
            <code>getInitialState</code>, which will be defined later and assigns the initial value to all 4
            variables defined above.
             */
            DataState state = getInitialState(0.05,0.0,0.0,0.0);

            /*
            We define the <code>TimedSystem</code> <code>system</code>, which will be the starting configuration from
            which the evolution sequence will be constructed.
            This configuration consists of 4 elements:
            - the controller <code>controller</code> defined above,
            - the data state <code>state</state> defined above,
            - a random function over data states, which implements interface <code>DataStateFunction</code> and maps a
            random generator <code>rg</code> and a data state <code>ds</code> to the data state obtained by updating
            <code>ds</code> with the list of changes given by method <code>selectAndApplyReaction/code>. Essentially,
            this static method, defined later, selects the next reaction among the 3 available according to Gillespie
            algorithm and realises the changes on variables that are consequence of the firing of the selected reaction,
            i.e. reactants are removed from <code>ds</code> and products are added to <code>ds</code>.
            - an expression over data states, which implements interface <code>DataStateExpression</code> and maps a
            data state <code>ds</code> to the time of next reaction.
             */
            RandomGenerator rand = new DefaultRandomGenerator();

            TimedSystem system = new TimedSystem(controller, (rg, ds) -> ds.apply(selectAndApplyReaction(rg, ds)), state, ds->selectReactionTime(rand,ds));


            /*

            EVOLUTION SEQUENCES

            Having the initial configuration <code>system</code>, we can generate its behaviour, which means that
            we can generate an evolution sequence.

             */

            /*
            Variable <code>size</code> gives the number of runs that are used to obtain the evolution sequence.
            More in detail, an evolution sequence, modelled by class <code>EvolutionSequence</code>, is a sequence of
            sample sets of system configurations, where configurations are modelled by class <code>TimedSystem</code>
            and sample sets by class <code>SampleSet</code>.
            In this context, <code>size</code> is the cardinality of those sample sets.
            */

            int size = 1;

            /*
            The evolution sequence <code>sequence></code> created by the following instruction consists in a sequence of
            sample sets of configurations of cardinality <code>size</size>, where the first sample of the list consists
            in <code>size</size> copies of configuration <code>system</code> defined above.
            Notice that <code>sequence></code> contains initially only this first sample, the subsequent ones will be
            created "on demand".
             */
            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, size);


            /*
            Each expression in the following list <code>F</code> allows us to read the value of a given variable
            from a data state
             */
            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(X));
            F.add(ds->ds.get(Y1));
            F.add(ds->ds.get(Y2));
            F.add(ds->ds.get(Z));
            F.add(ds->ds.getTimeReal());
            F.add(ds->ds.getTimeDelta());



            ArrayList<String> L = new ArrayList<>();
            L.add("X       ");
            L.add("Y1     ");
            L.add("Y2      ");
            L.add("Z      ");
            L.add("time      ");
            L.add("delta      ");



            /*

            EXPERIMENTS

            We propose some experiments showing how our tool can be used.

            */




            /*

            USING THE SIMULATOR

            We start with generating two evolution sequences from configuration <code>system</system>.
            Both evolution sequences are sequences of length <code>N</code> of sample sets of cardinality
            <code>size</code> of configurations, with the first sample set consisting in <code>size</code> copies of
            <code>system</code>.
            The second evolution sequence is perturbed by applying the perturbation returned by the static method
            <code>pertY (x)</code> defined later. Essentially, the method returns a perturbation that
            cuts a portion of the molecules of Y1 at a given step. In the prey/predator interpretation by Volterra, this
            corresponts to simulate an epidemy affecting the former population.
            For both evolution sequences, we store in .csv files some information allowing us to observe the dynamics of
            both the nominal and the perturbed system: for each time unit in [0,N-1] and for each variable, we store
            the average value that the variable assumes in the <code>size</code> configurations in the sample set
            obtained at that time unit.

            */

            System.out.println();
            System.out.println("Simulation of nominal and perturbed system");
            System.out.println();


            int N = 300;    // length ot the evolution sequence




            double[][] plot_LX = new double[N][1];
            double[][] plot_LY1 = new double[N][1];
            double[][] plot_LY2 = new double[N][1];
            double[][] plot_LZ = new double[N][1];

            double[][] data = SystemState.sample(rand, F, system, N, size);
            for (int i = 0; i<N; i++){
                plot_LX[i][0] = data[i][0];
                plot_LY1[i][0] = data[i][1];
                plot_LY2[i][0] = data[i][2];
                plot_LZ[i][0] = data[i][3];
            }
            Util.writeToCSV("./new_plotLX.csv",plot_LX);
            Util.writeToCSV("./new_plotLY1.csv",plot_LY1);
            Util.writeToCSV("./new_plotLY2.csv",plot_LY2);
            Util.writeToCSV("./new_plotLZ.csv",plot_LZ);

            double[][] plot_pertLX = new double[N][1];
            double[][] plot_pertLY1 = new double[N][1];
            double[][] plot_pertLY2 = new double[N][1];
            double[][] plot_pertLZ = new double[N][1];

            double[][] pertdata = SystemState.sample(rand, F, pertY1(N/2), system, N, size);
            for (int i = 0; i<N; i++){
                plot_pertLX[i][0] = pertdata[i][0];
                plot_pertLY1[i][0] = pertdata[i][1];
                plot_pertLY2[i][0] = pertdata[i][2];
                plot_pertLZ[i][0] = pertdata[i][3];
            }
            Util.writeToCSV("./new_plotLXpert.csv",plot_pertLX);
            Util.writeToCSV("./new_plotLY1pert.csv",plot_pertLY1);
            Util.writeToCSV("./new_plotLY2pert.csv",plot_pertLY2);
            Util.writeToCSV("./new_plotLZpert.csv",plot_pertLZ);

            double[][] plot_LY1Y2 = new double[N][2];
            double[][] plot_pertLY1Y2 = new double[N][2];
            for (int i = 0; i<N; i++){
                plot_LY1Y2[i][0] = data[i][1];
                plot_LY1Y2[i][1] = data[i][2];
                plot_pertLY1Y2[i][0] = pertdata[i][1];
                plot_pertLY1Y2[i][1] = pertdata[i][2];
            }
            Util.writeToCSV("./new_plotLY1Y2.csv",plot_LY1Y2);
            Util.writeToCSV("./new_plotLY1Y2pert.csv",plot_pertLY1Y2);

            /*
            While in the previous lines of code the average values of variables obtained step-by-step are stored in
            .cvs files, the following portion of code allows us to print them.
            */


            System.out.println();
            System.out.println("Simulation of nominal system - data average values:");
            System.out.println();
            printAvgData(rand, L, F, system, N, size, 0, N);
            System.out.println();
            System.out.println("Simulation of perturbed system - data average values:");
            printAvgDataPerturbed(rand, L, F, system, N, size, 0, N, pertY1(N/2));





   /*

            ESTIMATING BEHAVIORAL DISTANCES BETWEEN NOMINAL AND PERTURBED EVOLUTION SEQUENCES


            Now we generate again a nominal and a perturbed evolution sequence, as above.
            Then, we quantify the differences between those evolutions sequences, which corresponds to quantifying the
            behavioural distance between the nominal and the perturbed system. The differences are quantified with
            respect to the amount Y1 and Y2.

             */


            /*
            In order to quantify the difference between two evolution sequences w.r.t. Yi, we need to define the
            difference between two configurations w.r.t. Yi: given two configurations with Yi=m and Yi=n, the
            difference between those configurations w.r.t. Yi is the value |m-n|, normalised wrt the maximal value that
            can be assumed by Yi, so that this difference is always in [0,1].
            Since we cannot know a priori which is the maximal value that can be assumed by Yi, we estimate it.

            Therefore, we start with estimating the maximal values that can be assumed by all variables.
            To this purpose, we generate a nominal and a perturbed evolution sequence of length 2N and collect the
            maximal values that are assumed by the variables in all configurations in all sample sets.
            */





            System.out.println("");

            System.out.println("Estimating behavioural differences between nominal and perturbed system");
            System.out.println("");


            System.out.println("");
            System.out.println("Simulation of nominal system - Data maximal values:");
            double[] dataMax = printMaxData(rand, L, F, system, N, size, 0, 2*N);
            System.out.println("");
            System.out.println("Simulation of perturbed system - Data maximal values:");
            System.out.println("");
            double[] dataMax_p = printMaxDataPerturbed(rand, L, F, system, N, size, 0, 2*N, pertY1(N/4));

            double normalisationY1 = Math.max(dataMax[Y1],dataMax_p[Y1])*1.1;
            double normalisationY2 = Math.max(dataMax[Y2],dataMax_p[Y2])*1.1;




            /*
            The following instruction allows us to create the evolution sequence <code>sequence_p</code>, which is
            obtained from the evolution sequence <code>sequence</code> by applying a perturbation, where:
            - as above, the perturbation is returned by the static method <code>pertY1()</code> defined later
            - the perturbation is applied at step N/2
            - the sample sets of configurations in <code>sequence_p</code> have a cardinality which corresponds to that
            of <code>sequence</code> multiplied by <code>scale>/code>
            */


            int scale=2;
            EvolutionSequence sequence_p = sequence.apply(pertY1(N/2),0,scale);


            /*
            The following lines of code first defines two atomic distances between evolution sequences, named
            <code>atomicYi</code> for i=1,2. Then, these distances are evaluated, time-point by time-point, over
            evolution sequence <code>sequence</code> and its perturbed version <code>sequence_p</code> defined above.
            Finally, the time-point to time-point values of the distances are stored in .csv files.
            Technically, <code>distanceYi</code> is an atomic distance in the sense that it is an instance of
            class <code>AtomicDistanceExpression</code>, which consists in a data state expression,
            which maps a data state to a number, or rank, and a binary operator. As already discussed, in this case,
            given two configurations, the data state expression allow us to get the normalised value of Yi,
            which is a value in [0,1], from both configuration, and the binary operator gives us their difference, which,
            intuitively, is the difference with respect to the level of Zi between the two configurations.
            This distance will be lifted to two sample sets of configurations, those obtained from <code>sequence</code> and
            <code>sequence_p</code> at the same step.
            */

            int leftBound = 0;
            int rightBound = N;


            AtomicDistanceExpression atomicY1 = new AtomicDistanceExpression(ds->ds.get(Y1)/normalisationY1,(v1, v2) -> Math.abs(v2-v1));

            AtomicDistanceExpression atomicY2 = new AtomicDistanceExpression(ds->ds.get(Y2)/normalisationY2,(v1, v2) -> Math.abs(v2-v1));


            double[][] direct_evaluation_atomic_Y1 = new double[rightBound-leftBound][1];
            double[][] direct_evaluation_atomic_Y2 = new double[rightBound-leftBound][1];

            for (int i = 0; i<(rightBound-leftBound); i++){
                direct_evaluation_atomic_Y1[i][0] = atomicY1.compute(i+leftBound, sequence, sequence_p);
                direct_evaluation_atomic_Y2[i][0] = atomicY2.compute(i+leftBound, sequence, sequence_p);
            }

            Util.writeToCSV("./new_Latomic_Y1.csv",direct_evaluation_atomic_Y1);
            Util.writeToCSV("./new_Latomic_Y2.csv",direct_evaluation_atomic_Y2);





        } catch (RuntimeException e) {
            e.printStackTrace();
        }



    }





    /*
    The following method generates an evolution sequence consisting of a sequence of <code>steps</code> sample sets
    of cardinality <code>size</code>, with the first sample set consisting in <code>size</code> copies of configuration
    <code>s</code>.
    For each sample set, all expressions over data states in <code>F</code> are evaluated on all configurations and
    their average value are printed out.
    The method returns the average evaluation that each expression in <code>F</code> gets in all configurations in all
    sample sets that are in the sequence in between positions <code>leftbound</code> and <code>rightbound</code>
     */
    private static double[] printAvgData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound){
        System.out.println(label);
        /*
        The following instruction creates an evolution sequence consisting in a sequence of <code>steps</code> sample
        sets of cardinality <size>.
        The first sample set contains <code>size</code> copies of configuration <code>s</code>.
        The subsequent sample sets are derived by simulating the dynamics.
        For each step from 1 to <code>steps</code> and for each variable, the average value taken by the
        variables in the elements of the sample set at each step are printed out.
         */
        double[][] data_avg = SystemState.sample(rg, F, s, steps, size);
        double[] tot = new double[F.size()];
        Arrays.fill(tot, 0);
        for (int i = 0; i < data_avg.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data_avg[i].length -1 ; j++) {
                System.out.printf("%f   ", data_avg[i][j]);
                if (leftbound <= i & i <= rightbound) {
                    tot[j]=tot[j]+data_avg[i][j];
                }
            }
            System.out.printf("%f\n", data_avg[i][data_avg[i].length -1]);
            if (leftbound <= i & i <= rightbound) {
                tot[data_avg[i].length -1]=tot[data_avg[i].length -1]+data_avg[i][data_avg[i].length -1];
            }
        }
        System.out.println(" ");
        System.out.println("Avg over all steps of the average values taken in the single step by the variables:");
        for(int j=0; j<tot.length-1; j++){
            System.out.printf("%f   ", tot[j] / (rightbound-leftbound));
        }
        System.out.printf("%f\n", tot[tot.length-1]/ (rightbound-leftbound));
        System.out.println();
        System.out.println();
        return tot;
    }


    private static double[] printAvgDataPerturbed(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound, Perturbation perturbation){
        System.out.println(label);

        double[] tot = new double[F.size()];

        double[][] data_avg = SystemState.sample(rg, F, perturbation, s, steps, size);
        Arrays.fill(tot, 0);
        for (int i = 0; i < data_avg.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data_avg[i].length -1 ; j++) {
                System.out.printf("%f   ", data_avg[i][j]);
                if (leftbound <= i & i <= rightbound) {
                    tot[j]=tot[j]+data_avg[i][j];
                }
            }
            System.out.printf("%f\n", data_avg[i][data_avg[i].length -1]);
            if (leftbound <= i & i <= rightbound) {
                tot[data_avg[i].length -1]=tot[data_avg[i].length -1]+data_avg[i][data_avg[i].length -1];
            }
        }
        System.out.println();
        System.out.println("Avg over all steps of the average values taken in the single step by the variables:");
        for(int j=0; j<tot.length-1; j++){
            System.out.printf("%f   ", tot[j] / (rightbound-leftbound));
        }
        System.out.printf("%f\n", tot[tot.length-1]/ (rightbound-leftbound));
        System.out.println();
        return tot;

    }

    /*
    The following method generates an evolution sequence consisting of a sequence of <code>steps</code> sample sets
    of cardinality <code>size</code>, with the first sample set consisting in <code>size</code> copies of configuration
    <code>s</code>. For each configuration in each sample set, all expressions over data states in <code>F</code> are
    evaluated. the method returns the max evaluation that each expression in <code>F</code> gives in all sample sets
    that are in the sequence in between positions <code>leftbound</code> and <code>rightbound</code>
     */


    private static double[] printMaxData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound){

        /*
        The following instruction creates an evolution sequence consisting in a sequence of <code>steps</code> sample
        sets of cardinality <size>.
        The first sample set contains <code>size</code> copies of configuration <code>s</code>.
        The subsequent sample sets are derived by simulating the dynamics.
        Finally, for each step from 1 to <code>steps</code> and for each variable, the maximal value taken by the
        variable in the elements of the sample set is stored.
         */
        double[][] data_max = SystemState.sample_max(rg, F, s, steps, size);
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
        System.out.println();
        System.out.println();
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
        System.out.println();
        return max;

    }























    /*
    The following method selects the time of next reaction according to Gillespie algorithm.
     */
    public static double selectReactionTime(RandomGenerator rg, DataState state){
        double rate = 0.0;
        double[] lambda = new double[3];
        for (int j=0; j<3; j++){
            double weight = 1.0;
            for (int i=0; i<4; i++){
                if(r_input[j][i] > 0) {
                    weight = weight * Math.pow(state.get(i), r_input[j][i]);
                }
            }
            lambda[j] = r_k[j] * weight;
            rate = rate + lambda[j];
        }

        double rand = rg.nextDouble();
        return (1/rate)*Math.log(1/rand);
    }


    /*
    The following method selects the next reaction, according to Gillespie's algorithm, and returns the updates that
    allow for modifying the data state accordingly: these updates will remove the reactants used by the selected reaction
    from the data state, will add the products, and will tune the rate constant of promoters' activation according to the
    new value of proteins.
    */

    public static List<DataStateUpdate> selectAndApplyReaction(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double[] lambda = new double[3];
        double[] lambdaParSum = new double[3];
        double lambdaSum = 0.0;

        for (int j=0; j<3; j++){
            double weight = 1.0;
            for (int i=0; i<3; i++){
                weight = weight * Math.pow(state.get(i),r_input[j][i]);
            }
            lambda[j] = r_k[j] * weight;
            lambdaSum = lambda[j]+lambdaSum;
            lambdaParSum[j] = lambdaSum;
        }

        if(lambdaSum > 0){

            double token = 1 - rg.nextDouble();

            int selReaction = 0;

            while (lambdaParSum[selReaction] < token * lambdaSum) {
                selReaction++;
            }

            selReaction++;

            switch(selReaction){
                case 1:
                    for (int i=0; i<4; i++) {
                        double newArity = state.get(i) + r1_output[i] - r1_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 2:
                    for (int i = 0; i < 4; i++) {
                        double newArity = state.get(i) + r2_output[i] - r2_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 3:
                    for (int i = 0; i < 4; i++) {
                        double newArity = state.get(i) + r3_output[i] - r3_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
            }

        } else {
            System.out.println("Missing reagents");
        }


        return updates;

    }










    /*
    The following method returns a perturbation. In particular this is an atomic perturbation, that
    prunes at most half of the molecules of Y1.

     */


    public static Perturbation pertY1(int step){
        return new AtomicPerturbation(step,(rg,ds)->ds.apply(cutY1(rg,ds)));
    }
    private static List<DataStateUpdate> cutY1(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(Y1,state.get(Y1)*0.5));
        // updates.add(new DataStateUpdate(Y2,state.get(Y2)*0.5));
        return updates;
    }










    /*
    Method getInitialState assigns the initial value to all variables.
    The values are taken from "Ulysse Herbach: ''Harissa: Stochastic Simulation and Inference of Gene Regulatory Networks Based on Transcriptional
    Bursting''. Proc. CMSB 2023".

     */
    public static DataState getInitialState(double gran, double Tstep, double Treal, double Tdelta) {
        Map<Integer, Double> values = new HashMap<>();

        values.put(X, 1000.0);
        values.put(Y1, 1000.0);
        values.put(Y2, 1000.0);
        values.put(Z, 0.0);


        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Treal, Tdelta);
    }

}
