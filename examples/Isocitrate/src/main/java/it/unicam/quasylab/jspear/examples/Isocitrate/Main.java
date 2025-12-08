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

package it.unicam.quasylab.jspear.examples.Isocitrate;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.NilController;
import it.unicam.quasylab.jspear.distance.AtomicDistanceExpression;
import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.distance.MaxIntervalDistanceExpression;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.perturbation.AtomicPerturbation;
import it.unicam.quasylab.jspear.perturbation.IterativePerturbation;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.*;

public class Main {

    /* The isocitrate dehydrogenase regulatory system (IDHKPIDH) of E. Coli controls the partitioning of
    carbon flux, and it is useful when the bacterium of E. coli grows on substances, like for example
    acetate, which contains only a small quantity of carbon. Without this regulation system, in fact,
    the organism would not have enough carbon available for biosynthesis of cell constituents.
    */

    // LIST OF ALL REACTIONS

    public static final int[] r1_input =  {1,0,1,0,0}; // reactants: E and Ip
    public static final int[] r1_output = {0,0,0,1,0}; // product: EIp
    public static final double r1_k = 0.02;

    public static final int[] r2_input =  {0,0,0,1,0}; // reactants: EIp
    public static final int[] r2_output = {1,0,1,0,0}; // product: E and Ip
    public static final double r2_k = 0.5;

    public static final int[] r3_input =  {0,0,0,1,0}; // reactants: EIp
    public static final int[] r3_output = {1,1,0,0,0}; // product: E and I
    public static final double r3_k = 0.5;

    public static final int[] r4_input =  {0,1,0,1,0}; // reactants: I and EIp
    public static final int[] r4_output = {0,0,0,0,1}; // product: EIpI
    public static final double r4_k = 0.02;

    public static final int[] r5_input =  {0,0,0,0,1}; // reactants: EIpI
    public static final int[] r5_output = {0,1,0,1,0}; // product: I and EIp
    public static final double r5_k = 0.5;

    public static final int[] r6_input =  {0,0,0,0,1}; // reactants: EIpI
    public static final int[] r6_output = {0,0,1,1,0}; // product: Ip and EIp
    public static final double r6_k = 0.1;

    /*
      Interesting question: is I robust to variations of E and Ip?
      Our answer strategy: we perturb a system by changing the values for E and Ip and we check whether the distance
      that depends on the value of I between the nominal system and the perturbed one is below a given threshold.
    */

    public static final int[][] r_input = {r1_input,r2_input,r3_input,r4_input,r5_input,r6_input};
    public static final int[][] r_output = {r1_output,r2_output,r3_output,r4_output,r5_output,r6_output};

    public static final double[] r_k = {r1_k,r2_k,r3_k,r4_k,r5_k,r6_k};


    // LIST OF SPECIES
    public static final int E = 0;  // enzyme able to phosporylate and dephosporylate I
    public static final int I = 1;  // isocitrate dehydrogenase, IDH, which regulate the carbon flux
    public static final int Ip = 2; // IDH in phosporilated form,
    public static final int EIp = 3; // compound E + Ip.
    public static final int EIpI = 4; // compound EIp + I
    private static final int NUMBER_OF_VARIABLES = 5;

    public static final double THRESHOLD = 0.03;

    public static final int LEFT_BOUND = 400;

    public static final int RIGHT_BOUND = 1000;


    // MAIN PROGRAM
    public static void main(String[] args) throws IOException {
        try {

            RandomGenerator rand = new DefaultRandomGenerator();

            int size = 100;

            Controller controller = new NilController();

            /*
            The <code>DataState</code> <code>state</code> will be the initial state of our system.
            The value of variables is assigned by function <code>getInitialState</state>, which assigns to each variable
            a random value chosen in a suitable interval.
             */

            DataState state = getInitialState(rand,1.0,0.0,0.0,0.0);

            /*
            The <code>TimedSystem</code> <code>system</code> is a system starting in state <code>state</code>.
            Its evolution is determined by Gillespie's algorithm:
            <code>selectReactionTime</code> is a static method that selects the time of next reaction according to Gillespie algorithm.
            <code>selectAndApplyReaction</code> is a static method that selects the reaction among the available ones and modifies the state accordingly.
            */

            TimedSystem system = new TimedSystem(controller, (rg, ds) -> ds.apply(selectAndApplyReaction(rg, ds)), state, ds->selectReactionTime(rand,ds));

            /*
            The <code>EvolutionSequence</code> <code>sequence</code> originates from <code>size</code> copies of <code>system</code>
            */

            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, size);

            EvolutionSequence sequence_001_100 = sequence.apply(pertEandIp(0.001,100),0,10);
            EvolutionSequence sequence_001_500 = sequence.apply(pertEandIp(0.001,500),0,10);
            EvolutionSequence sequence_10_1000 = sequence.apply(pertEandIp(10,1000),0,10);
            EvolutionSequence sequence_1_500 = sequence.apply(pertEandIp(1,500),0,10);
            EvolutionSequence sequence_10_100 = sequence.apply(pertEandIp(10,100),0,10);
            EvolutionSequence sequence_50_300 = sequence.apply(pertEandIp(50,300),0,10);


            /*
            Given the system <code>system</code>, the following instructions simulate <code>size</code> runs consisting in
            <code>RIGHT_BOUND</code> steps. At each step, the average value taken by each variable in the runs is printed out.
            Then, for each variable the min and max value that are printed in the steps in the interval
            [LEFT_BOUND,RIGHT_BOUND] are stored in arrays minMax[0] AND minMax[1], respectively.
            Then, the same simulation is done for <code>psystem</code>.
            Therefore, we can observe an evolution of both the nominal and the perturbed system.
            */

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(E));
            F.add(ds->ds.get(I));
            F.add(ds->ds.get(Ip));
            F.add(ds->ds.get(EIp));
            F.add(ds->ds.get(EIpI));
            F.add(ds->ds.getTimeDelta());
            F.add(ds->ds.getTimeReal());
            ArrayList<String> L = new ArrayList<>();
            L.add("E");
            L.add("I");
            L.add("Ip");
            L.add("EIp");
            L.add("EIpI");

            double[][] minMax = printLMinMaxData(rand, L, F, system, RIGHT_BOUND, size, LEFT_BOUND, RIGHT_BOUND);

            double[][] minMax_001_100 = printPerturbed(rand, L, F, system, RIGHT_BOUND, size, LEFT_BOUND, RIGHT_BOUND,pertEandIp(0.001,100));
            double[][] minMax_001_500 = printPerturbed(rand, L, F, system, RIGHT_BOUND, size, LEFT_BOUND, RIGHT_BOUND,pertEandIp(0.001,500));
            double[][] minMax_10_1000 = printPerturbed(rand, L, F, system, RIGHT_BOUND, size, LEFT_BOUND, RIGHT_BOUND,pertEandIp(10,1000));

            double normalisation_001_100 = Math.max(minMax[1][I],minMax_001_100[1][I])*1.1;
            double normalisation_001_500 = Math.max(minMax[1][I],minMax_001_500[1][I])*1.1;
            double normalisation_10_1000 = Math.max(minMax[1][I],minMax_10_1000[1][I])*1.1;

            double[][] minMax_1_500 = printPerturbed(rand, L, F, system, RIGHT_BOUND, size, LEFT_BOUND, RIGHT_BOUND,pertEandIp(1,500));
            double[][] minMax_10_100 = printPerturbed(rand, L, F, system, RIGHT_BOUND, size, LEFT_BOUND, RIGHT_BOUND,pertEandIp(10,100));
            double[][] minMax_50_300 = printPerturbed(rand, L, F, system, RIGHT_BOUND, size, LEFT_BOUND, RIGHT_BOUND,pertEandIp(50,300));

            double normalisation_1_500 = Math.max(minMax[1][I],minMax_1_500[1][I])*1.1;
            double normalisation_10_100 = Math.max(minMax[1][I],minMax_10_100[1][I])*1.1;
            double normalisation_50_300 = Math.max(minMax[1][I],minMax_50_300[1][I])*1.1;

            /*
            Definition of distance expressions evaluating the Wasserstein distance between the
            distributions reached, at a given time step, in the nominal system and the perturbed one,
            each considering the proper normalisation factor according to the perturbation that is applied.
            */

            DistanceExpression atomic_001_100 = new AtomicDistanceExpression(ds->ds.get(I)/normalisation_001_100,(v1, v2) -> Math.abs(v2-v1));

            DistanceExpression atomic_001_500 = new AtomicDistanceExpression(ds->ds.get(I)/normalisation_001_500,(v1, v2) -> Math.abs(v2-v1));

            DistanceExpression atomic_10_1000 = new AtomicDistanceExpression(ds->ds.get(I)/normalisation_10_1000,(v1, v2) -> Math.abs(v2-v1));

            DistanceExpression atomic_1_500 = new AtomicDistanceExpression(ds->ds.get(I)/normalisation_1_500,(v1, v2) -> Math.abs(v2-v1));

            DistanceExpression atomic_10_100 = new AtomicDistanceExpression(ds->ds.get(I)/normalisation_10_100,(v1, v2) -> Math.abs(v2-v1));

            DistanceExpression atomic_50_300 = new AtomicDistanceExpression(ds->ds.get(I)/normalisation_50_300,(v1, v2) -> Math.abs(v2-v1));

            /*
            Extension in time of the atomic distances.
            We consider the maximum over the interval [400,1000]
            */

            DistanceExpression distance_001_100 = new MaxIntervalDistanceExpression(
                    atomic_001_100,
                    400,
                    RIGHT_BOUND);

            DistanceExpression distance_001_500 = new MaxIntervalDistanceExpression(
                    atomic_001_500,
                    400,
                    RIGHT_BOUND);

            DistanceExpression distance_10_1000 = new MaxIntervalDistanceExpression(
                    atomic_10_1000,
                    400,
                    RIGHT_BOUND);

            DistanceExpression distance_1_500 = new MaxIntervalDistanceExpression(
                    atomic_1_500,
                    400,
                    RIGHT_BOUND);

            DistanceExpression distance_10_100 = new MaxIntervalDistanceExpression(
                    atomic_10_100,
                    400,
                    RIGHT_BOUND);

            DistanceExpression distance_50_300 = new MaxIntervalDistanceExpression(
                    atomic_50_300,
                    400,
                    RIGHT_BOUND);

            // Pointwise evaluation of distances

            System.out.println("Starting evaluation of distances");

            double[][] direct_evaluation_001_100 = new double[1000][1];
            double[][] direct_evaluation_001_500 = new double[1000][1];
            double[][] direct_evaluation_10_1000 = new double[1000][1];
            double[][] direct_evaluation_1_500 = new double[1000][1];
            double[][] direct_evaluation_10_100 = new double[1000][1];
            double[][] direct_evaluation_50_300 = new double[1000][1];


            for (int i = 0; i<1000; i++){
                direct_evaluation_001_100[i][0] = atomic_001_100.compute(i+200, sequence, sequence_001_100);
                direct_evaluation_001_500[i][0] = atomic_001_500.compute(i+200, sequence, sequence_001_500);
                direct_evaluation_10_1000[i][0] = atomic_10_1000.compute(i+200, sequence, sequence_10_1000);

                direct_evaluation_1_500[i][0] = atomic_1_500.compute(i+200, sequence, sequence_1_500);
                direct_evaluation_10_100[i][0] = atomic_10_100.compute(i+200, sequence, sequence_10_100);
                direct_evaluation_50_300[i][0] = atomic_50_300.compute(i+200, sequence, sequence_50_300);
            }

            Util.writeToCSV("./atomic_001_100.csv",direct_evaluation_001_100);
            Util.writeToCSV("./atomic_001_500.csv",direct_evaluation_001_500);
            Util.writeToCSV("./atomic_10_1000.csv",direct_evaluation_10_1000);
            Util.writeToCSV("./atomic_1_500.csv",direct_evaluation_1_500);
            Util.writeToCSV("./atomic_10_100.csv",direct_evaluation_10_100);
            Util.writeToCSV("./atomic_50_300.csv",direct_evaluation_50_300);

            System.out.println("Evaluation of distances completed");

            // ROBUSTNESS FORMULAE

            /* The following formula tells us whether the difference expressed by <code>distance</cod> between the nominal
            system and its version perturbed by <code>pertEandIp</code> is bound by THRESHOLD.
            The result of the evaluation of the formula is printed out by the subsequent two lines of code.
            */

            RobustnessFormula robF_001_100 = new AtomicRobustnessFormula(pertEandIp(0.001,100),
                    distance_001_100,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    THRESHOLD);

            RobustnessFormula robF_001_500 = new AtomicRobustnessFormula(pertEandIp(0.001,500),
                    distance_001_500,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    THRESHOLD);

            RobustnessFormula robF_10_1000 = new AtomicRobustnessFormula(pertEandIp(10,1000),
                    distance_10_1000,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    THRESHOLD);

            RobustnessFormula robF_1_500 = new AtomicRobustnessFormula(pertEandIp(1,500),
                    distance_1_500,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    THRESHOLD);

            RobustnessFormula robF_10_100 = new AtomicRobustnessFormula(pertEandIp(10,100),
                    distance_10_100,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    THRESHOLD);

            RobustnessFormula robF_50_300 = new AtomicRobustnessFormula(pertEandIp(50,300),
                    distance_50_300,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    THRESHOLD);

            System.out.println("Starting evaluation of formulae");

            TruthValues value1 = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(robF_001_100).eval(5, 0, sequence);
            System.out.println("\n robF_001_100 evaluation at 0: " + value1);

            TruthValues value2 = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(robF_001_500).eval(5, 0, sequence);
            System.out.println("\n robF_001_500 evaluation at 0: " + value2);

            TruthValues value3 = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(robF_10_1000).eval(5, 0, sequence);
            System.out.println("\n robF_10_1000 evaluation at 0: " + value3);

            TruthValues value4 = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(robF_1_500).eval(5, 0, sequence);
            System.out.println("\n robF_001_100 evaluation at 0: " + value4);

            TruthValues value5 = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(robF_10_100).eval(5, 0, sequence);
            System.out.println("\n robF_001_500 evaluation at 0: " + value5);

            TruthValues value6 = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(robF_50_300).eval(5, 0, sequence);
            System.out.println("\n robF_10_1000 evaluation at 0: " + value6);


            System.out.println("Evaluation of formulae completed");


        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }


    /* This method will simulate <code>size</code> runs of length <code>steps</code> of system <code>s</code>.
       For each time step, each data state expression in list <code>F</code> is evaluated on all <code>s</code>
       systems and the average value is printed.
       The method returns a double[][], where:
       - double[1,j] contains the maximum value of jth expression in F
    */

    private static double[][] printLMinMaxData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound){
        double[][] result = new double[2][NUMBER_OF_VARIABLES];
        System.out.println(label);
        double[][] data_av = SystemState.sample(rg, F, s, steps, size);
        double[][] data_max = SystemState.sample_max(rg, F, s, steps, size);
        double[] max = new double[NUMBER_OF_VARIABLES];
        Arrays.fill(max, Double.NEGATIVE_INFINITY);
        for (int i = 0; i < data_av.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data_av[i].length -1 ; j++) {
                System.out.printf("%f   ", data_av[i][j]);
                if (j<NUMBER_OF_VARIABLES & leftbound <= i & i <= rightbound) {
                    if (max[j] < data_max[i][j]) {
                        max[j] = data_max[i][j];
                        result[1][j]=data_max[i][j];
                    }
                }
            }
            System.out.printf("%f\n", data_av[i][data_av[i].length -1]);
        }
        System.out.printf("%s   ", "max:");
        for(int j=0; j<NUMBER_OF_VARIABLES-1; j++){
            System.out.printf("%f   ", max[j]);
        }
        System.out.printf("%f\n", max[NUMBER_OF_VARIABLES-1]);
        return result;
    }


    private static double[][] printPerturbed(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound, Perturbation perturbation){
        double[][] result = new double[2][NUMBER_OF_VARIABLES];
        System.out.println(label);
        double[][] data_av = SystemState.sample(rg, F, perturbation, s, steps, size);
        double[][] data_max = SystemState.sample_max(rg, F, perturbation, s, steps, size);
        double[] max = new double[NUMBER_OF_VARIABLES];
        Arrays.fill(max, Double.NEGATIVE_INFINITY);
        for (int i = 0; i < data_av.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data_av[i].length -1 ; j++) {
                System.out.printf("%f   ", data_av[i][j]);
                if (j<NUMBER_OF_VARIABLES & leftbound <= i & i <= rightbound) {
                    if (max[j] < data_max[i][j]) {
                        max[j] = data_max[i][j];
                        result[1][j]=data_max[i][j];
                    }
                }
            }
            System.out.printf("%f\n", data_av[i][data_av[i].length -1]);
        }
        System.out.printf("%s   ", "max:");
        for(int j=0; j<NUMBER_OF_VARIABLES-1; j++){
            System.out.printf("%f   ", max[j]);
        }
        System.out.printf("%f\n", max[NUMBER_OF_VARIABLES-1]);
        return result;
    }


    /* PERTURBATIONS:
    Perturbation <code>pertEandIp</code> perturbs the system state by applying function <code>changeEandIp</code> at the
    first computation step.
    It modifies the initial concentration of species E and Ip, by adding
    x molecules of E, and
    y molecules of Ip.
    */

    public static Perturbation pertEandIp(double x, double y){
        return new AtomicPerturbation(0, (rg,ds)->ds.apply(changeEandIp(rg,ds,x,y)));
    }

    private static List<DataStateUpdate> changeEandIp(RandomGenerator rg, DataState state, double x, double y) {
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(E,  x));
        updates.add(new DataStateUpdate(Ip,  y));
        return updates;
    }

    /*
    The following method selects the time of next reaction, according to Gillespie's algorithm.
    */

    public static double selectReactionTime(RandomGenerator rg, DataState state){
        double rate = 0.0;
        double[] lambda = new double[6];
        for (int j=0; j<6; j++){
            double weight = 1.0;
            for (int i=0; i<5; i++){
                if(r_input[j][i] > 0) {
                    weight = weight * Math.pow(state.get(i), r_input[j][i]);
                }
            }
            lambda[j] = r_k[j] * weight;
            rate = rate + lambda[j];
        }

        double rand = rg.nextDouble();
        double t = (1/rate)*Math.log(1/rand);
        return t;
    }


    /*
    The following method selects the next reaction, according to Gillespie's algorithm,
    and returns the updates that allow for modifying the state accordingly.
    */

    public static List<DataStateUpdate> selectAndApplyReaction(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double[] lambda = new double[6];
        double[] lambdaParSum = new double[6];
        double lambdaSum = 0.0;

        for (int j=0; j<6; j++){
            double weight = 1.0;
            for (int i=0; i<5; i++){
                weight = weight * Math.pow(state.get(i),r_input[j][i]);
            }
            lambda[j] = r_k[j]* weight;
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

            if (selReaction == 1) {
                for (int i = 0; i < 5; i++) {
                    double newArity = state.get(i) + r1_output[i] - r1_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 2) {
                for (int i = 0; i < 5; i++) {
                    double newArity = state.get(i) + r2_output[i] - r2_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 3) {
                for (int i = 0; i < 5; i++) {
                    double newArity = state.get(i) + r3_output[i] - r3_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 4) {
                for (int i = 0; i < 5; i++) {
                    double newArity = state.get(i) + r4_output[i] - r4_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 5) {
                for (int i = 0; i < 5; i++) {
                    double newArity = state.get(i) + r5_output[i] - r5_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 6) {
                for (int i = 0; i < 5; i++) {
                    double newArity = state.get(i) + r6_output[i] - r6_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }
        } else {
            System.out.println("Missing reagents");
        }

        return updates;

    }

    // INITIALISATION OF DATA STATE. The initial value for all variables are randomly chosen between 1 and 100.

    public static DataState getInitialState(RandomGenerator rand, double gran, double Tstep, double Treal, double Tdelta) {
        Map<Integer, Double> values = new HashMap<>();

        double initE = Math.ceil(100 * rand.nextDouble());
        double initI = Math.ceil(100 * rand.nextDouble());
        double initIp = Math.ceil(100 * rand.nextDouble());
        double initEIp = Math.ceil(100 * rand.nextDouble());
        double initEIpI = Math.ceil(100 * rand.nextDouble());
        values.put(E, initE);
        values.put(I, initI);
        values.put(Ip, initIp);
        values.put(EIp, initEIp);
        values.put(EIpI, initEIpI);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Treal, Tdelta);
    }

}
