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

package it.unicam.quasylab.jspear.examples.lotka;



import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.NilController;
import it.unicam.quasylab.jspear.distance.*;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.perturbation.AtomicPerturbation;
import it.unicam.quasylab.jspear.perturbation.IterativePerturbation;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.perturbation.SequentialPerturbation;
import it.unicam.quasylab.jspear.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.math.BigDecimal;
import java.util.*;

public class Main {

    /*
    General Introduction to Lotka reactions.


    */



    /*

    LOTKA REACTIONS

     X + Y1  ---c1---> 2Y1
     Y1 + Y2 ---c2---> 2Y2
     Y2      ---c3---> Z


    */


    /*

    ARRAYS REPRESENTING REACTANTS/PRODUCTS OF REACTIONS

    Technically, for each of the 3 reactions, we define 2 arrays.
    Each of these arrays has 4 positions, which are associated to the four species.
    In particular, the 4 positions are for: X, Y1, Y2, Z.
    Then, the two arrays for each reaction ri, with i=1,..,4, are:
    - ri_input: position j is 1 if the variable corresponding to position j is a reactant of the reaction
    - ro_output: position j is 1 if the variable corresponding to j is a product of the reaction
    The arrays are defined below.

    */


    /*
    Reaction r1 is X + Y1 ---c1---> 2Y1
    */
    public static final int[] r1_input = {1,1,0,0};
    public static final int[] r1_output = {0,2,0,0};

    /*
    Reaction r2 is Y1 + Y2 ---c2---> 2Y2.
    */
    public static final int[] r2_input =  {0,1,1,0};
    public static final int[] r2_output = {0,0,2,0};

    /*
    Reaction r3 is Y2 ---c3---> Z.
    */
    public static final int[] r3_input =  {0,0,1,0};
    public static final int[] r3_output = {0,0,0,1};


    public static final int[][] r_input = {r1_input,r2_input,r3_input};



    /*

    VARIABLES MODELING THE STATUS OF THE SYSTEM

    Below a list of 30 variables, the idea being that the value of these 30 variables gives a data state, namely an
    instance of class <code>DataState</code> representing the status of all quantities of the system.
    We have variables for active and inactive promoters, mRNA, proteins and reaction rates. Reaction rates can vary
    since promoter parameters, i.e. the rates of promoter activation and deactivation, depend on the amount of proteins.
    Each variable is associated with an index, from 0 to 29.
    */
    public static final int X = 1000; // amount of molecules of X
    public static final int Y1 = 1000; // amount of molecules of Y1.
    public static final int Y2 = 1000; // amount of molecules of Y2.
    public static final int Z = 3; // amount of molecules of Z.


    public static final int c1 = 0.01; // rate constant r1
    public static final int c2 = 0.01; // rate constant r1
    public static final int c3 = 10; // rate constant r1

    private static final int NUMBER_OF_VARIABLES = 7;



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
            <code>getInitialState</code>, which will be defined later and assigns the initial value to all 30
            variables defined above.
             */
            DataState state = getInitialState(1.0,0.0,0.0,0.0);

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


            ArrayList<String> L = new ArrayList<>();
            L.add("X       ");
            L.add("Y1     ");
            L.add("Y2      ");
            L.add("Z      ");



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
            <code>itZ1TranslRate(x)</code> defined later. Essentially, the method returns a cyclic perturbation that
            affects the translation rate  of gene 1:  for <code>replica</code> times, it has no effect for the first w1
            time points, i.e., the system behaves regularly, then in the subsequent <code>w2</code> time points, the
            translation rare is decremented by x, which impacts directly on the evolution of <code>Z1</code> and,
            through interactions, on <code>Z2</code> and <code>Z3</code>.
            This perturbation models protein translation deregulation.

            For both evolution sequences, we store in .csv files some information allowing us to observe the dynamics of
            both the nominal and the perturbed system: for each time unit in [0,N-1] and for each variable, we store
            the average value that the variable assumes in the <code>size</code> configurations in the sample set
            obtained at that time unit.

            */

            System.out.println("");
            System.out.println("Simulation of nominal and perturbed system");
            System.out.println("");


            int N = 30;    // length ot the evolution sequence

            // double x = -3.0; //


            // int w1=50;
            // int w2=50;
            // int replica= 5;



            double[][] plot_X = new double[N][1];
            double[][] plot_Y1 = new double[N][1];
            double[][] plot_Y2 = new double[N][1];
            double[][] plot_Z = new double[N][1];

            double[][] data = SystemState.sample(rand, F, system, N, size);
            for (int i = 0; i<N; i++){
                plot_X[i][0] = data[i][0];
                plot_Y1[i][0] = data[i][1];
                plot_Y2[i][0] = data[i][2];
                plot_Z[i][0] = data[i][3];
            }
            Util.writeToCSV("./new_plotX.csv",plot_X);
            Util.writeToCSV("./new_plotY1.csv",plot_Y1);
            Util.writeToCSV("./new_plotY2.csv",plot_Y2);
            Util.writeToCSV("./new_plotZ.csv",plot_Z);

/*
            double[][] pdata = SystemState.sample(rand, F, itZ1TranslRate(x,w1,w2,replica), system, N, size);
            for (int i = 0; i<N; i++){
                plot_pz1[i][0] = pdata[i][3];
                plot_pz2[i][0] = pdata[i][7];
                plot_pz3[i][0] = pdata[i][11];

                plot_px1[i][0] = pdata[i][2];
                plot_px2[i][0] = pdata[i][6];
                plot_px3[i][0] = pdata[i][10];
            }
            Util.writeToCSV("./new_pplotZ1.csv",plot_pz1);
            Util.writeToCSV("./new_pplotZ2.csv",plot_pz2);
            Util.writeToCSV("./new_pplotZ3.csv",plot_pz3);

            Util.writeToCSV("./new_pplotX1.csv",plot_px1);
            Util.writeToCSV("./new_pplotX2.csv",plot_px2);
            Util.writeToCSV("./new_pplotX3.csv",plot_px3);

*/



            /*
            While in the previous lines of code the average values of variables obtained step-by-step are stored in
            .cvs files, the following portion of code allows us to print them.
            */


            System.out.println("");
            System.out.println("Simulation of nominal system - data average values:");
            System.out.println("");
            printAvgData(rand, L, F, system, N, size, 0, N);
            System.out.println("");
            //System.out.println("Simulation of perturbed system - data average values:");
            //System.out.println("");
            //printAvgDataPerturbed(rand, L, F, system, N, size, 0, N, itZ1TranslRate(x, w1, w2, replica));






            /*

            ESTIMATING BEHAVIORAL DISTANCES BETWEEN NOMINAL AND PERTURBED EVOLUTION SEQUENCES


            Now we generate again a nominal and a perturbed evolution sequence, as above.
            Then, we quantify the differences between those evolutions sequences, which corresponds to quantifying the
            behavioural distance between the nominal and the perturbed system. The differences are quantified with
            respect to the amount of protein Z1, Z2 or Z3.

             */


            /*
            In order to quantify the difference between two evolution sequences w.r.t. Zi, we need to define the
            difference between two configurations w.r.t. Zi: given two configurations with Zi=m and Zi=n, the
            difference between those configurations w.r.t. Zi is the value |m-n|, normalised wrt the maximal value that
            can be assumed by Zi, so that this difference is always in [0,1].
            Since we cannot know a priori which is the maximal value that can be assumed by Zi, we estimate it.

            Therefore, we start with estimating the maximal values that can be assumed by all variables.
            To this purpose, we generate a nominal and a perturbed evolution sequence of length 2N and collect the
            maximal values that are assumed by the variables in all configurations in all sample sets.
            */


            /*
            System.out.println("");
            System.out.println("Estimating behavioural differences between nominal and perturbed system");
            System.out.println("");


            System.out.println("");
            System.out.println("Simulation of nominal system - Data maximal values:");
            double[] dataMax = printMaxData(rand, L, F, system, N, size, w1+w2, 2*N);
            System.out.println("");
            System.out.println("Simulation of perturbed system - Data maximal values:");
            System.out.println("");
            double[] dataMax_p = printMaxDataPerturbed(rand, L, F, system, N, size, w1+w2, 2*N, itZ1TranslRate(x,w1,w2,replica));

            double normalisationZ1 = Math.max(dataMax[Z1],dataMax_p[Z1])*1.1;
            double normalisationZ2 = Math.max(dataMax[Z2],dataMax_p[Z1])*1.1;
            double normalisationZ3 = Math.max(dataMax[Z3],dataMax_p[Z2])*1.1;


            */

            /*
            The following instruction allows us to create the evolution sequence <code>sequence_p</code>, which is
            obtained from the evolution sequence <code>sequence</code> by applying a perturbation, where:
            - as above, the perturbation is returned by the static method <code>itZ1PertRate()</code> defined later
            - the perturbation is applied at step 0
            - the sample sets of configurations in <code>sequence_p</code> have a cardinality which corresponds to that
            of <code>sequence</code> multiplied by <code>scale>/code>
            */


            int scale=5;
            EvolutionSequence sequence_p = sequence.apply(itZ1TranslRate(x, w1, w2, replica),0,scale);


            /*
            The following lines of code first defines three atomic distances between evolution sequences, named
            <code>atomicZi</code> for i=1,2,3. Then, these distances are evaluated, time-point by time-point, over
            evolution sequence <code>sequence</code> and its perturbed version <code>sequence_p</code> defined above.
            Finally, the time-point to time-point values of the distances are stored in .csv files.
            Technically, <code>distanceZi</code> is an atomic distance in the sense that it is an instance of
            class <code>AtomicDistanceExpression</code>, which consists in a data state expression,
            which maps a data state to a number, or rank, and a binary operator. As already discussed, in this case,
            given two configurations, the data state expression allow us to get the normalised value of protein Zi,
            which is a value in [0,1], from both configuration, and the binary operator gives us their difference, which,
            intuitively, is the difference with respect to the level of Zi between the two configurations.
            This distance will be lifted to two sample sets of configurations, those obtained from <code>sequence</code> and
            <code>sequence_p</code> at the same step.
            */



            /*
            int leftBound = 0;
            int rightBound = 30;


            AtomicDistanceExpression atomicZ1 = new AtomicDistanceExpression(ds->ds.get(Z1)/normalisationZ1,(v1, v2) -> Math.abs(v2-v1));

            AtomicDistanceExpression atomicZ2 = new AtomicDistanceExpression(ds->ds.get(Z2)/normalisationZ2,(v1, v2) -> Math.abs(v2-v1));

            AtomicDistanceExpression atomicZ3 = new AtomicDistanceExpression(ds->ds.get(Z3)/normalisationZ3,(v1, v2) -> Math.abs(v2-v1));

            double[][] direct_evaluation_atomic_Z1 = new double[rightBound-leftBound][1];
            double[][] direct_evaluation_atomic_Z2 = new double[rightBound-leftBound][1];
            double[][] direct_evaluation_atomic_Z3 = new double[rightBound-leftBound][1];

            for (int i = 0; i<(rightBound-leftBound); i++){
                direct_evaluation_atomic_Z1[i][0] = atomicZ1.compute(i+leftBound, sequence, sequence_p);
                direct_evaluation_atomic_Z2[i][0] = atomicZ2.compute(i+leftBound, sequence, sequence_p);
                direct_evaluation_atomic_Z3[i][0] = atomicZ3.compute(i+leftBound, sequence, sequence_p);
            }

            Util.writeToCSV("./atomic_Z1.csv",direct_evaluation_atomic_Z1);
            Util.writeToCSV("./atomic_Z2.csv",direct_evaluation_atomic_Z2);
            Util.writeToCSV("./atomic_Z3.csv",direct_evaluation_atomic_Z3);





           */



            /*
            USING THE MODEL CHECKER

            Later, we will write down a robustness formula that simply expresses whether the maximal of these distances is
            below a given threshold.
            First we define the distances <code>distanceZi</code>, as instances of <code>MaxIntervalDistanceExpression</code>.
            Each <code>distanceZi</code> evaluates <code>atomicZi</code> in all time-points and returns the max value.
            Then we define the distance expression <code>distanceMaxZ1Z2Z3</code>, which returns the maximal value
            among those returned by three distances defined above.
            Then, we define a robustness formula, in particular an atomic formula, namely an instance of
            <code>AtomicRobustnessFormula</code>.
            This formula will be evaluated on the evolution sequence <code>sequence</code> and expresses that the
            distance, expressed by expression distance <code>distanceMaxZ1Z2Z3</code> between that evolution
            sequence and the evolution sequence obtained from it by applying the perturbation returned by method
            <code>itZ1TranslRate(x)</code>, is below a given threshold.

             */


            /*
            int leftRBound=800;
            int rightRBound=900;

            DistanceExpression dMax = new MaxDistanceExpression(
                    atomicZ1,
                    new MaxDistanceExpression(atomicZ2, atomicZ3)
            );

            DistanceExpression intdMax = new MaxIntervalDistanceExpression(
                    dMax,
                    leftRBound,
                    rightRBound
            );



            double[][] robEvaluations = new double[20][2];
            RobustnessFormula robustF;
            int index=0;
            double thresholdB = 1;
            for(int i = 0; i < 20 ; i++){
                double threshold = thresholdB + i;
                threshold = threshold / 100;
                robustF = new AtomicRobustnessFormula(itZ1TranslRate(x,w1,w2,replica),
                        intdMax,
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        threshold);
                TruthValues value = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(robustF).eval(5, 0, sequence);
                System.out.println(" ");
                System.out.println("\n robustF evaluation at " + threshold + ": " + value);
                robEvaluations[index][1]=value.valueOf();
                robEvaluations[index][0]=threshold;
                index++;
            }
            Util.writeToCSV("./evalR.csv",robEvaluations);




             */







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
        System.out.println("");
        System.out.println("");
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
        System.out.println("");
        System.out.println("Avg over all steps of the average values taken in the single step by the variables:");
        for(int j=0; j<tot.length-1; j++){
            System.out.printf("%f   ", tot[j] / (rightbound-leftbound));
        }
        System.out.printf("%f\n", tot[tot.length-1]/ (rightbound-leftbound));
        System.out.println("");
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




    private static List<DataStateUpdate> upd_s11(RandomGenerator rg, DataState state, double x){
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(s11,Math.max(state.get(s11)+x,0)));
        return updates;
    }


    /*
    The following method returns a perturbation. In particular this is an iterative perturbation, i.e. an instance of
    <code>IterativePerturbation</code>. It consists of two elements, an integer and a body perturbation, the idea being
    that the integer expresses how many times the body perturbation is applied.
    In this case, the body perturbation is a sequential perturbation, namely an instance of
    <code>SequentialPerturbation</code>, which, essentially, consists in two perturbations where the second is applied
    after the first has terminated its effect. The first perturbation is applied after <code>w1<code> steps
    and, increments the translation rate of Z1 by the parameter <code>x</code>. The second perturbation is applied after
    further <code>w2<code> stepps and reverts the effects of the first one.

     */


    public static Perturbation itZ1TranslRate(double x, int w1, int w2, int replica){
        return new IterativePerturbation(replica,
                new SequentialPerturbation(
                        new AtomicPerturbation(w1,(rg,ds)->ds.apply(upd_s11(rg,ds,x))),
                        new AtomicPerturbation(w2,(rg,ds)->ds.apply(upd_s11(rg,ds,-x)))
                )
        );
    }









    /*
    The following method returns a perturbation. In particular this is an iterative perturbation, i.e. an instance of
    <code>IterativePerturbation</code>. It consists of two elements, an integer and a body perturbation, the idea being
    that the integer expresses how many times the body perturbation is applied.
    In this case, the body perturbation is a sequential perturbation, namely an instance of
    <code>SequentialPerturbation</code>, which, essentially, consists in two perturbations where the second is applied
    after the first has terminated its effect. The first perturbation is applied after 10 steps
    and, increments the translation rate of Z2 and decrements that of Z1. The second perturbation is applied after 40
    steps and reverts the effects of the first one. Overall, the perturbed system evolves with perturbed values of the
    degradation rates of Z1 and Z1 in between time intervals [10,50], [60,100], [110.150].

     */











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
            lambda[j] = state.get(j+4) * weight;
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
            for (int i=0; i<14; i++){
                weight = weight * Math.pow(state.get(i),r_input[j][i]);
            }
            lambda[j] = state.get(j+4) * weight;
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
    Method getInitialState assigns the initial value to all variables.
    The values are taken from "Ulysse Herbach: ''Harissa: Stochastic Simulation and Inference of Gene Regulatory Networks Based on Transcriptional
    Bursting''. Proc. CMSB 2023".

     */
    public static DataState getInitialState(double gran, double Tstep, double Treal, double Tdelta) {
        Map<Integer, Double> values = new HashMap<>();

        // the system starts with all promoters inactive, no mRNA molecule and no protein

        values.put(G1, 1.0);
        values.put(AG1, 0.0);
        values.put(X1, 0.0);
        values.put(Z1, 0.0);

        values.put(G2, 1.0);
        values.put(AG2, 0.0);
        values.put(X2, 0.0);
        values.put(Z2, 0.0);

        values.put(G3, 1.0);
        values.put(AG3, 0.0);
        values.put(X3, 0.0);
        values.put(Z3, 0.0);

        double initial_kon1 = K01 + K11 * Math.exp(BETA1)/(1+Math.exp(BETA1));

        values.put(kon1, initial_kon1);
        values.put(koff1, 5.0);
        values.put(s01, 250.0);
        values.put(s11, 7.0);
        values.put(d01, 1.0);
        values.put(d11, 0.1);

        double initial_kon2 = K02 + K12 * Math.exp(BETA2)/(1+Math.exp(BETA2));

        values.put(kon2, initial_kon2);
        values.put(koff2, 5.0);
        values.put(s02, 250.0);
        values.put(s12, 7.0);
        values.put(d02, 1.0);
        values.put(d12, 0.1);

        double initial_kon3 = K03 + K13 * Math.exp(BETA3)/(1+Math.exp(BETA3));

        values.put(kon3, initial_kon3);
        values.put(koff3, 5.0);
        values.put(s03, 250.0);
        values.put(s13, 7.0);
        values.put(d03, 1.0);
        values.put(d13, 0.1);


        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Treal, Tdelta);
    }

}