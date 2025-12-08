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

package it.unicam.quasylab.jspear.examples.envzompr;

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


    // The specification of the 11 reactions:

    public static final int[] r1_input = {0,0,0,0,0,0,0,1};
    public static final int[] r1_output = {1,0,0,0,0,0,0,0};
    public static final double r1_k = 0.5;

    public static final int[] r2_input = {1,0,0,0,0,0,0,0};
    public static final int[] r2_output = {0,0,0,0,0,0,0,1};
    public static final double r2_k = 0.5;

    public static final int[] r3_input = {0,0,1,0,0,0,0,0};
    public static final int[] r3_output = {1,0,0,0,0,0,0,0};
    public static final double r3_k = 0.5;

    public static final int[] r4_input = {1,0,0,0,0,0,0,0};
    public static final int[] r4_output = {0,0,1,0,0,0,0,0};
    public static final double r4_k = 0.5;

    public static final int[] r5_input = {0,0,1,0,0,0,0,0};
    public static final int[] r5_output = {0,0,0,1,0,0,0,0};
    public static final double r5_k = 0.1;

    public static final int[] r6_input = {0,1,0,1,0,0,0,0};
    public static final int[] r6_output = {0,0,0,0,1,0,0,0};
    public static final double r6_k = 0.02;

    public static final int[] r7_input = {0,0,0,0,1,0,0,0};
    public static final int[] r7_output = {0,1,0,1,0,0,0,0};
    public static final double r7_k = 0.5;

    public static final int[] r8_input = {0,0,0,0,1,0,0,0};
    public static final int[] r8_output = {1,0,0,0,0,1,0,0};
    public static final double r8_k = 0.5;

    public static final int[] r9_input = {0,0,0,0,0,1,0,1};
    public static final int[] r9_output = {0,0,0,0,0,0,1,0};
    public static final double r9_k = 0.02;

    public static final int[] r10_input = {0,0,0,0,0,0,1,0};
    public static final int[] r10_output = {0,0,0,0,0,1,0,1};
    public static final double r10_k = 0.5;

    public static final int[] r11_input = {0,0,0,0,0,0,1,0};
    public static final int[] r11_output = {0,1,0,0,0,0,0,1};
    public static final double r11_k = 0.1;

    public static final int[][] r_input = {r1_input,r2_input,r3_input,r4_input,r5_input,r6_input,r7_input,r8_input,r9_input,r10_input,r11_input};
    public static final int[][] r_output = {r1_output,r2_output,r3_output,r4_output,r5_output,r6_output,r7_output,r8_output,r9_output,r10_output,r11_output};

    public static final double[] r_k = {r1_k,r2_k,r3_k,r4_k,r5_k,r6_k,r7_k,r8_k,r9_k,r10_k,r11_k};

    public static final int H = 600;

    public static final double ETA = 0.15;


    // The 8 variables, one for each species

    public static final int X = 0;
    public static final int Y = 1;
    public static final int XT = 2;
    public static final int XP = 3;
    public static final int XPY = 4;
    public static final int YP = 5;
    public static final int XDYP = 6;
    public static final int XD = 7;


    private static final int NUMBER_OF_VARIABLES = 8;



    // MAIN PROGRAM
    public static void main(String[] args) throws IOException {
        try {

            RandomGenerator rand = new DefaultRandomGenerator();

            int size = 30;

            Controller controller = new NilController();

            DataState state = getInitialState(rand,1.0,0.0,0.0,0.0);
            DataState state10_50 = getInitialState10_50(rand,1.0,0.0,0.0,0.0);
            DataState state250_1000 = getInitialState250_1000(rand,1.0,0.0,0.0,0.0);

            TimedSystem system = new TimedSystem(controller, (rg, ds) -> ds.apply(selectAndApplyReaction(rg, ds)), state, ds->selectReactionTime(new DefaultRandomGenerator(),ds));
            TimedSystem system10_50 = new TimedSystem(controller, (rg, ds) -> ds.apply(selectAndApplyReaction(rg, ds)), state10_50, ds->selectReactionTime(new DefaultRandomGenerator(),ds));
            TimedSystem system250_1000 = new TimedSystem(controller, (rg, ds) -> ds.apply(selectAndApplyReaction(rg, ds)), state250_1000, ds->selectReactionTime(new DefaultRandomGenerator(),ds));

            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, size);
            EvolutionSequence sequence_10_50 = sequence.apply(addXY(10,50),0,10);
            EvolutionSequence sequence_250_1000 = sequence.apply(addXY(250,1000),0,10);
            EvolutionSequence sequence_100_400 = sequence.apply(addXY(100,400),0,10);
            EvolutionSequence sequence_150_100 = sequence.apply(addXY(150,100),0,10);
            EvolutionSequence sequence_500_200 = sequence.apply(addXY(500,200),0,10);
            EvolutionSequence sequence_70_30 = sequence.apply(addXY(70,30),0,10);

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(X));
            F.add(ds->ds.get(Y));
            F.add(ds->ds.get(XT));
            F.add(ds->ds.get(XP));
            F.add(ds->ds.get(XPY));
            F.add(ds->ds.get(YP));
            F.add(ds->ds.get(XDYP));
            F.add(ds->ds.get(XD));
            F.add(ds->ds.getTimeDelta());
            F.add(ds->ds.getTimeReal());
            ArrayList<String> L = new ArrayList<>();
            L.add("X");
            L.add("Y");
            L.add("XT");
            L.add("XP");
            L.add("XPY");
            L.add("YP");
            L.add("XDYP");
            L.add("XD");

            double[][] minMax = printLMinMaxData(new DefaultRandomGenerator(), L, F, system, H+1, size, 0, H+1);

            double[][] minMax10_50 = printLMinMaxData(new DefaultRandomGenerator(), L, F, system10_50, H+1, size, 0, H+1);

            double[][] minMax250_1000 = printLMinMaxData(new DefaultRandomGenerator(), L, F, system250_1000, H+1, size, 0, H+1);

            DistanceExpression atomica = new AtomicDistanceExpression(ds->ds.get(YP)/(minMax[1][YP]*1.1-minMax[0][YP]*0.9),(v1, v2) -> Math.abs(v2-v1));

            /*
            double[][] direct_evaluation_25_150 = new double[600][1];
            double[][] direct_evaluation_250_1000 = new double[600][1];

            for (int i = 0; i<600; i++){
                direct_evaluation_25_150[i][0] = atomica.compute(i, sequence, sequence_25_150);
                direct_evaluation_250_1000[i][0] = atomica.compute(i, sequence, sequence_250_1000);
            }

            for (int i = 0; i<600; i++){
                System.out.println(direct_evaluation_25_150[i][0]);
            }

            for (int i = 0; i<600; i++){
                System.out.println(direct_evaluation_250_1000[i][0]);
            }

             */

            // Pointwise evaluation of distances

            double[][] direct_evaluation_10_50 = new double[600][1];
            double[][] direct_evaluation_250_1000 = new double[600][1];
            double[][] direct_evaluation_100_400 = new double[600][1];
            double[][] direct_evaluation_150_100 = new double[600][1];
            double[][] direct_evaluation_500_200 = new double[600][1];
            double[][] direct_evaluation_70_30 = new double[600][1];

            for (int i = 0; i<600; i++){
                direct_evaluation_10_50[i][0] = atomica.compute(i, sequence, sequence_10_50);
                direct_evaluation_250_1000[i][0] = atomica.compute(i, sequence, sequence_250_1000);
                direct_evaluation_100_400[i][0] = atomica.compute(i, sequence, sequence_100_400);
                direct_evaluation_150_100[i][0] = atomica.compute(i, sequence, sequence_150_100);
                direct_evaluation_500_200[i][0] = atomica.compute(i, sequence, sequence_500_200);
                direct_evaluation_70_30[i][0] = atomica.compute(i, sequence, sequence_70_30);
            }

            Util.writeToCSV("./10_50.csv",direct_evaluation_10_50);
            Util.writeToCSV("./250_1000.csv",direct_evaluation_250_1000);
            Util.writeToCSV("./100_400.csv",direct_evaluation_100_400);
            Util.writeToCSV("./150_100.csv",direct_evaluation_150_100);
            Util.writeToCSV("./500_200.csv",direct_evaluation_500_200);
            Util.writeToCSV("./70_30.csv",direct_evaluation_70_30);

            // Formulae for robustness

            DistanceExpression distance = new MaxIntervalDistanceExpression(
                    new AtomicDistanceExpression(ds->ds.get(YP)/(minMax[1][YP]*1.1-minMax[0][YP]*0.9),(v1, v2) -> Math.abs(v2-v1)),
                    200,
                    600
            );

            RobustnessFormula robustness_10_50 = new AtomicRobustnessFormula(
                    addXY(10,50),
                    distance,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA
            );

            RobustnessFormula robustness_250_1000 = new AtomicRobustnessFormula(
                    addXY(250,1000),
                    distance,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA
            );

            RobustnessFormula robustness_100_400 = new AtomicRobustnessFormula(
                    addXY(100,400),
                    distance,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA
            );

            RobustnessFormula robustness_150_100 = new AtomicRobustnessFormula(
                    addXY(150,100),
                    distance,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA
            );

            RobustnessFormula robustness_500_200 = new AtomicRobustnessFormula(
                    addXY(500,200),
                    distance,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA
            );

            RobustnessFormula robustness_70_30 = new AtomicRobustnessFormula(
                    addXY(70,30),
                    distance,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    ETA
            );

            RobustnessFormula always_10_50 = new AlwaysRobustnessFormula(
                    robustness_10_50,
                    0,
                    300
            );

            RobustnessFormula always_250_1000 = new AlwaysRobustnessFormula(
                    robustness_250_1000,
                    0,
                    300
            );

            RobustnessFormula always_100_400 = new AlwaysRobustnessFormula(
                    robustness_100_400,
                    0,
                    300
            );

            RobustnessFormula always_150_100 = new AlwaysRobustnessFormula(
                    robustness_150_100,
                    0,
                    300
            );

            RobustnessFormula always_500_200 = new AlwaysRobustnessFormula(
                    robustness_500_200,
                    0,
                    300
            );

            RobustnessFormula always_70_30 = new AlwaysRobustnessFormula(
                    robustness_70_30,
                    0,
                    300
            );


            BooleanSemanticsVisitor BoolEvaluator = new BooleanSemanticsVisitor();

            System.out.println("Evaluation of always_10_50 at step "+0+": "+BoolEvaluator.eval(always_10_50).eval(size, 0, sequence));
            System.out.println("Evaluation of always_250_1000 at step "+0+": "+BoolEvaluator.eval(always_250_1000).eval(size, 0, sequence));
            System.out.println("Evaluation of always_100_40 at step "+0+": "+BoolEvaluator.eval(always_100_400).eval(size, 0, sequence));
            System.out.println("Evaluation of always_150_100 at step "+0+": "+BoolEvaluator.eval(always_150_100).eval(size, 0, sequence));
            System.out.println("Evaluation of always_500_200 at step "+0+": "+BoolEvaluator.eval(always_500_200).eval(size, 0, sequence));
            System.out.println("Evaluation of always_70_30 at step "+0+": "+BoolEvaluator.eval(always_70_30).eval(size, 0, sequence));


        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }


    public static Perturbation addXY(int x, int y){
        return new AtomicPerturbation(0, (rg,ds)->ds.apply(changeXandY(rg,ds,x,y)));
    }

    private static List<DataStateUpdate> changeXandY(RandomGenerator rg, DataState state, int x, int y) {
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(X,  x));
        updates.add(new DataStateUpdate(Y,  y));
        return updates;
    }

    private static double[][] printLMinMaxData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size, int leftbound, int rightbound){


        double[][] result = new double[2][NUMBER_OF_VARIABLES];
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, s, steps, size);
        double[] min = new double[NUMBER_OF_VARIABLES];
        double[] max = new double[NUMBER_OF_VARIABLES];
        for(int i = 0; i < NUMBER_OF_VARIABLES ; i++){
            min[i]=Double.POSITIVE_INFINITY;
            max[i]=Double.NEGATIVE_INFINITY;
        }
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1 ; j++) {
                System.out.printf("%f   ", data[i][j]);
                if (j<NUMBER_OF_VARIABLES & leftbound <= i & i <= rightbound) {
                    if (min[j] > data[i][j]) {
                        min[j] = data[i][j];
                        result[0][j] = data[i][j];
                    }
                    if (max[j] < data[i][j]) {
                        max[j] = data[i][j];
                        result[1][j]=data[i][j];
                    }
                }
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }

        System.out.printf("%s   ", "min:");
        for(int j=0; j<NUMBER_OF_VARIABLES-1; j++){
            System.out.printf("%f   ", min[j]);
        }
        System.out.printf("%f\n", min[NUMBER_OF_VARIABLES-1]);

        System.out.printf("%s   ", "max:");
        for(int j=0; j<NUMBER_OF_VARIABLES-1; j++){
            System.out.printf("%f   ", max[j]);
        }
        System.out.printf("%f\n", max[NUMBER_OF_VARIABLES-1]);
        return result;
    }







    /*
    The following method selects the time of next reaction, according to Gillespie's algorithm.
    */

    public static double selectReactionTime(RandomGenerator rg, DataState state){
        double rate = 0.0;
        double[] lambda = new double[11];
        for (int j=0; j<r_input.length; j++){
            double weight = 1.0;
            for (int i=0; i<NUMBER_OF_VARIABLES; i++){
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
    The following method selects the next reaction, according to Gillespie's algorithm, and returns the updates that allow for
     modifying the state accordingly.
    */

    public static List<DataStateUpdate> selectAndApplyReaction(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double[] lambda = new double[r_input.length];
        double[] lambdaParSum = new double[r_input.length];
        double lambdaSum = 0.0;

        for (int j=0; j<r_input.length; j++){
            double weight = 1.0;
            for (int i=0; i<NUMBER_OF_VARIABLES; i++){
                if(r_input[j][i] >0) {
                    if(state.get(i) < r_input[j][i]){
                        weight = 0;
                    }
                    else {
                        weight = weight * Math.pow(state.get(i), r_input[j][i]);
                    }
                }
            }
            lambda[j] = r_k[j] * weight;
            lambdaSum = lambda[j]+lambdaSum;
            lambdaParSum[j] = lambdaSum;
        }

        if(lambdaSum > 0){

            double token = 1 - rg.nextDouble();

            int selReaction = 0;

            while (lambdaParSum[selReaction] <= token * lambdaSum) {
                selReaction++;
            }

            selReaction++;

            if (selReaction == 1) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r1_output[i] - r1_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 2) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r2_output[i] - r2_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 3) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r3_output[i] - r3_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 4) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r4_output[i] - r4_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 5) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r5_output[i] - r5_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 6) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r6_output[i] - r6_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 7) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r7_output[i] - r7_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 8) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r8_output[i] - r8_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 9) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r9_output[i] - r9_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 10) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r10_output[i] - r10_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

            if (selReaction == 11) {
                for (int i = 0; i < NUMBER_OF_VARIABLES; i++) {
                    double newArity = state.get(i) + r11_output[i] - r11_input[i];
                    updates.add(new DataStateUpdate(i, newArity));
                }
            }

        }
        else {
            System.out.println("esausto");
        }

        return updates;


    }

    // INITIALISATION OF DATA STATE. The initial value for all variables are randomly chosen between 1 and 100.


    public static DataState getInitialState(RandomGenerator rand, double gran, double Tstep, double Treal, double Tdelta) {
        Map<Integer, Double> values = new HashMap<>();
        values.put(X, 25.0);
        values.put(Y, 150.0);
        values.put(XT, 0.0);
        values.put(XP, 0.0);
        values.put(XPY, 0.0);
        values.put(YP, 10.0);
        values.put(XDYP, 0.0);
        values.put(XD, 50.0);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Treal, Tdelta);
    }

    public static DataState getInitialState10_50(RandomGenerator rand, double gran, double Tstep, double Treal, double Tdelta) {
        Map<Integer, Double> values = new HashMap<>();
        values.put(X, 10.0);
        values.put(Y, 50.0);
        values.put(XT, 0.0);
        values.put(XP, 0.0);
        values.put(XPY, 0.0);
        values.put(YP, 10.0);
        values.put(XDYP, 0.0);
        values.put(XD, 50.0);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Treal, Tdelta);
    }

    public static DataState getInitialState250_1000(RandomGenerator rand, double gran, double Tstep, double Treal, double Tdelta) {
        Map<Integer, Double> values = new HashMap<>();
        values.put(X, 250.0);
        values.put(Y, 1000.0);
        values.put(XT, 0.0);
        values.put(XP, 0.0);
        values.put(XPY, 0.0);
        values.put(YP, 10.0);
        values.put(XDYP, 0.0);
        values.put(XD, 50.0);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Treal, Tdelta);
    }


}
