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

package it.unicam.quasylab.jspear.examples.multipleSclerosis;



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

    public static final int[] r1_input = {0,0,0,0,0,0,0,0,0,0,0};
    public static final int[] r1_output = {0,1,0,0,0,0,0,0,0,0,0};

    public static final int[] r2_input =  {0,0,0,0,0,0,0,0,0,0,0};
    public static final int[] r2_output = {0,0,0,1,0,0,0,0,0,0,0};

    public static final int[] r3_input =  {1,0,0,0,0,0,0,0,0,0,0};
    public static final int[] r3_output = {0,1,0,0,0,0,0,0,0,0,0};

    public static final int[] r4_input =  {0,1,0,0,0,0,0,0,0,0,0};
    public static final int[] r4_output = {1,0,0,0,0,0,0,0,0,0,0};

    public static final int[] r5_input =  {0,0,1,0,0,0,0,0,0,0,0};
    public static final int[] r5_output = {0,0,0,1,0,0,0,0,0,0,0};

    public static final int[] r6_input =  {0,0,0,1,0,0,0,0,0,0,0};
    public static final int[] r6_output = {0,0,1,0,0,0,0,0,0,0,0};

    public static final int[] r7_input =  {0,1,0,0,0,0,0,0,0,0,0};
    public static final int[] r7_output = {0,0,0,0,0,0,0,0,0,0,0};

    public static final int[] r8_input =  {0,0,0,1,0,0,0,0,0,0,0};
    public static final int[] r8_output = {0,0,0,0,0,0,0,0,0,0,0};

    public static final int[] r9_input =  {2,0,0,0,0,0,0,0,0,0,0};
    public static final int[] r9_output = {0,0,0,0,1,0,0,0,0,0,0};

    public static final int[] r10_input =  {0,0,0,0,1,0,0,0,0,0,0};
    public static final int[] r10_output = {2,0,0,0,0,0,0,0,0,0,0};

    public static final int[] r11_input =  {0,0,2,0,0,0,0,0,0,0,0};
    public static final int[] r11_output = {0,0,0,0,0,1,0,0,0,0,0};

    public static final int[] r12_input =  {0,0,0,0,0,1,0,0,0,0,0};
    public static final int[] r12_output = {0,0,2,0,0,0,0,0,0,0,0};

    public static final int[] r13_input =  {0,0,1,0,0,0,1,0,0,0,0};
    public static final int[] r13_output = {0,0,0,0,0,0,0,1,0,0,0};

    public static final int[] r14_input =  {0,0,0,0,0,0,0,1,0,0,0};
    public static final int[] r14_output = {0,0,1,0,0,0,1,0,0,0,0};

    public static final int[] r15_input =  {1,0,0,0,0,1,0,0,0,0,0};
    public static final int[] r15_output = {1,0,2,0,0,0,0,0,0,0,0};

    public static final int[] r16_input =  {1,0,0,0,0,1,0,0,0,0,0};
    public static final int[] r16_output = {0,0,0,0,0,1,0,0,0,0,0};

    public static final int[] r17_input =  {0,0,1,0,1,0,0,0,0,0,0};
    public static final int[] r17_output = {2,0,1,0,0,0,0,0,0,0,0};

    public static final int[] r18_input =  {0,0,1,0,1,0,0,0,0,0,0};
    public static final int[] r18_output = {0,0,2,0,1,0,0,0,0,0,0};

    public static final int[] r19_input =  {0,0,1,0,0,0,0,0,0,0,0};
    public static final int[] r19_output = {0,0,0,0,0,0,0,0,0,0,0};

    public static final int[] r20_input =  {1,0,0,0,0,0,0,0,0,0,0};
    public static final int[] r20_output = {2,0,0,0,0,0,0,0,0,0,0};

    public static final int[] r21_input =  {1,0,1,0,0,0,0,0,0,0,0};
    public static final int[] r21_output = {0,0,0,0,0,0,1,0,0,0,0};

    public static final int[] r22_input =  {0,0,0,0,0,0,1,0,0,0,0};
    public static final int[] r22_output = {1,0,1,0,0,0,0,0,0,0,0};


    public static final int[] r23_input =  {0,0,0,0,0,0,0,0,1,0,0};
    public static final int[] r23_output = {0,0,0,0,0,0,0,0,1,1,0};

    public static final int[] r24_input =  {0,0,0,0,0,0,0,0,0,1,0};
    public static final int[] r24_output = {0,0,0,0,0,0,0,0,0,0,1};

    public static final int[] r25_input =  {0,0,0,0,0,0,0,0,0,1,0};
    public static final int[] r25_output = {0,0,0,0,0,0,0,0,0,0,0};

    public static final int[][] r_input = {r1_input,r2_input,r3_input,r4_input,r5_input,r6_input,r7_input,r8_input,r9_input,r10_input,
            r11_input,r12_input,r13_input,r14_input,r15_input,r16_input,r17_input,r18_input,r19_input,r20_input,
            r21_input,r22_input,r23_input,r24_input,r25_input};



    public static final int E = 0;
    public static final int Er = 1;
    public static final int R = 2;
    public static final int Rr = 3;

    public static final int E2 = 4;
    public static final int R2 = 5;
    public static final int ER = 6;
    public static final int ER2 = 7;

    public static final int Ea = 8;
    public static final int l = 9;
    public static final int L = 10;

    public static final int c1 = 11;
    public static final int c2 = 12;
    public static final int c3 = 13;
    public static final int c4 = 14;
    public static final int c5 = 15;
    public static final int c6 = 16;
    public static final int c7 = 17;
    public static final int c8 = 18;
    public static final int c9 = 19;
    public static final int c10 = 20;
    public static final int c11 = 21;
    public static final int c12 = 22;
    public static final int c13 = 23;
    public static final int c14 = 24;
    public static final int c15 = 25;
    public static final int c16 = 26;
    public static final int c17 = 27;
    public static final int c18 = 28;
    public static final int c19 = 29;
    public static final int c20 = 30;
    public static final int c21 = 31;
    public static final int c22 = 32;
    public static final int c23 = 33;
    public static final int c24 = 34;
    public static final int c25 = 35;

    private static final int NUMBER_OF_VARIABLES = 36;

    public static final double IE = 0.0;
    public static final double IR = 0.0;
    public static final double eta = 0.01;
    public static final double delta = 1.0;
    public static final double beta = 0.01;
    public static final double k1 = 1.0;
    public static final double k2 = 0.25;
    public static final double k3 = 0.1;
    public static final double alphaE = 1.5;
    public static final double alphaR = 1.0;
    public static final double gammaE = 0.2;
    public static final double gammaR = 0.2;
    public static final double kE = 1000.0;
    public static final double kR = 200.0;
    public static final double km1 = k1*Math.pow(kR,2) - gammaE;
    public static final double km2 = k2*kE - alphaR;
    public static final double km3 = k3*Math.pow(kR,2) - alphaE;
    public static final double d1 = 1.0;
    public static final double d2 = 0.02;
    public static final double r = 0.1;
    private static final double a = 22800.0;
    private static final double Einit = 1000.0;
    private static final double Rinit = 200.0;



    // MAIN PROGRAM
    public static void main(String[] args) throws IOException {
        try {

            Controller controller = new NilController();

            DataState state = getInitialState(1.0,0.0,0.0,0.0);

            RandomGenerator rand = new DefaultRandomGenerator();

            TimedSystem system = new TimedSystem(controller, (rg, ds) -> ds.apply(selectAndApplyReaction(rg, ds)), state, ds->selectReactionTime(rand,ds));

            int size = 50;

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(E));
            F.add(ds->ds.get(R));
            F.add(ds->ds.get(Er));
            F.add(ds->ds.get(Rr));
            F.add(ds->ds.get(E2));
            F.add(ds->ds.get(R2));
            F.add(ds->ds.get(ER));
            F.add(ds->ds.get(ER2));
            F.add(ds->ds.get(l));
            F.add(ds->ds.get(L));

            int steps = 2000;


            double[][] data_avg = SystemState.sample(rand, F, system, steps, size);
            for (int i = 0; i < data_avg.length; i++) {
                System.out.printf("%d>   ", i);
                for (int j = 0; j < data_avg[i].length -1 ; j++) {
                    System.out.printf("%f   ", data_avg[i][j]);
                }
                System.out.printf("%f\n", data_avg[i][data_avg[i].length -1]);
            }
            Util.writeToCSV("./multipleSclerosis.csv",data_avg);


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
    private static void printAvgData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size){
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
                //if (leftbound <= i & i <= rightbound) {
                //    tot[j]=tot[j]+data_avg[i][j];
                //}
            }
            System.out.printf("%f\n", data_avg[i][data_avg[i].length -1]);
            //if (leftbound <= i & i <= rightbound) {
            //    tot[data_avg[i].length -1]=tot[data_avg[i].length -1]+data_avg[i][data_avg[i].length -1];
            //}
        }
        //System.out.println(" ");
        //System.out.println("Avg over all steps of the average values taken in the single step by the variables:");
        //for(int j=0; j<tot.length-1; j++){
        //    System.out.printf("%f   ", tot[j] / (rightbound-leftbound));
        //}
        //System.out.printf("%f\n", tot[tot.length-1]/ (rightbound-leftbound));
        //System.out.println("");
        //System.out.println("");
        //return tot;
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

    /*
    The following method selects the time of next reaction according to Gillespie algorithm.
     */
    public static double selectReactionTime(RandomGenerator rg, DataState state){
        double rate = 0.0;
        double[] lambda = new double[25];
        for (int j=0; j<25; j++){
            double weight = 1.0;
            for (int i=0; i<11; i++){
                if(r_input[j][i] > 0) {
                    weight = weight * Math.pow(state.get(i), r_input[j][i]);
                }
            }
            lambda[j] = state.get(j+11) * weight;

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

        double[] lambda = new double[25];
        double[] lambdaParSum = new double[25];
        double lambdaSum = 0.0;

        for (int j=0; j<25; j++){
            double weight = 1.0;
            for (int i=0; i<11; i++){
                weight = weight * Math.pow(state.get(i),r_input[j][i]);
            }
            lambda[j] = state.get(j+11) * weight;

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
                    for (int i=0; i<11; i++) {
                        double newArity = state.get(i) + r1_output[i] - r1_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 2:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r2_output[i] - r2_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 3:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r3_output[i] - r3_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 4:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r4_output[i] - r4_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 5:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r5_output[i] - r5_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 6:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r6_output[i] - r6_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 7:
                    for (int i=0; i<11; i++) {
                        double newArity = state.get(i) + r7_output[i] - r7_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 8:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r8_output[i] - r8_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 9:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r9_output[i] - r9_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 10:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r10_output[i] - r10_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 11:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r11_output[i] - r11_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 12:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r12_output[i] - r12_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 13:
                    for (int i=0; i<11; i++) {
                        double newArity = state.get(i) + r13_output[i] - r13_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 14:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r14_output[i] - r14_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 15:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r15_output[i] - r15_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 16:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r16_output[i] - r16_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 17:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r17_output[i] - r17_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 18:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r18_output[i] - r18_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 19:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r19_output[i] - r19_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 20:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r20_output[i] - r20_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 21:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r21_output[i] - r21_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 22:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r22_output[i] - r22_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 23:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r23_output[i] - r23_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 24:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r24_output[i] - r24_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
                case 25:
                    for (int i = 0; i < 11; i++) {
                        double newArity = state.get(i) + r25_output[i] - r25_input[i];
                        updates.add(new DataStateUpdate(i, newArity));
                    }
                    break;
            }
        } else {
            System.out.println("Missing reagents");
        }

        return updates;

    }





    public static DataState getInitialState(double gran, double Tstep, double Treal, double Tdelta) {
        Map<Integer, Double> values = new HashMap<>();

        values.put(E, Einit);
        values.put(Er, 0.0);
        values.put(R, Rinit);
        values.put(Rr, 0.0);
        values.put(E2, 0.0);
        values.put(R2, 0.0);
        values.put(ER, 0.0);
        values.put(ER2, 0.0);
        values.put(Ea, Math.pow(Einit/a,2));
        values.put(l, 0.0);
        values.put(L, 0.0);

        values.put(c1, IE);
        values.put(c2, IR);
        values.put(c3, eta);
        values.put(c4, delta);
        values.put(c5, eta);
        values.put(c6, delta);
        values.put(c7, beta);
        values.put(c8, beta);
        values.put(c9, k2);
        values.put(c10, km2);
        values.put(c11, k1);
        values.put(c12, km1);
        values.put(c13, k3);
        values.put(c14, km3);
        values.put(c15, gammaE);
        values.put(c16, gammaE/Rinit);
        values.put(c17, alphaR);
        values.put(c18, alphaR/Einit);
        values.put(c19, gammaR);
        values.put(c20, alphaE/Rinit);//alphaE*Math.pow(kR,2));
        values.put(c21, k3);
        values.put(c22, km3);
        values.put(c23, d1);
        values.put(c24, d2);
        values.put(c25, r);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Treal, Tdelta);
    }

}
