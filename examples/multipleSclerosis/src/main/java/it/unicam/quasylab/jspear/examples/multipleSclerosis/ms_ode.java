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

public class ms_ode {

    public static final int E = 0;
    public static final int Er = 1;
    public static final int R = 2;
    public static final int Rr = 3;

    public static final int Ea = 4;
    public static final int l = 5;
    public static final int L = 6;

    private static final int NUMBER_OF_VARIABLES = 7;

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
    private static final double h = 5.0;

    public static void main(String[] args) throws IOException {
        try {

            Controller controller = new NilController();

            DataState state = getInitialState( );

            RandomGenerator rand = new DefaultRandomGenerator();

            ControlledSystem system = new ControlledSystem(controller, (rg, ds) -> ds.apply(odeEnv(rg, ds)), state);

            int size = 1;

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(E));
            F.add(ds->ds.get(R));
            F.add(ds->ds.get(Er));
            F.add(ds->ds.get(Rr));
            F.add(ds->ds.get(l));
            F.add(ds->ds.get(L));

            int steps = 2000;


            double[][] data_avg = SystemState.sample(rand, F, system, steps, size);
            double[][] E_values = new double[steps][1];
            double[][] R_values = new double[steps][1];
            double[][] Er_values = new double[steps][1];
            double[][] Rr_values = new double[steps][1];
            double[][] l_values= new double[steps][1];
            double[][] L_values = new double[steps][1];


            for (int i = 0; i < data_avg.length; i++) {
                System.out.printf("%d>   ", i);
                for (int j = 0; j < data_avg[i].length -1 ; j++) {
                    System.out.printf("%f   ", data_avg[i][j]);
                }
                System.out.printf("%f\n", data_avg[i][data_avg[i].length -1]);
            }

            for(int j = 0; j < steps; j++){
                E_values[j][0] = Math.log10(data_avg[j][0]);
                R_values[j][0] = Math.log10(data_avg[j][1]);
                Er_values[j][0] = Math.log10(data_avg[j][2]);
                Rr_values[j][0] = Math.log10(data_avg[j][3]);
                l_values[j][0] = Math.log10(data_avg[j][4]);
                L_values[j][0] = Math.log10(data_avg[j][5]);
            }

            Util.writeToCSV("./multipleSclerosisOde.csv",data_avg);
            Util.writeToCSV("./multipleSclerosisOdeE.csv",E_values);
            Util.writeToCSV("./multipleSclerosisOdeR.csv",R_values);
            Util.writeToCSV("./multipleSclerosisOdeEr.csv",Er_values);
            Util.writeToCSV("./multipleSclerosisOdeRr.csv",Rr_values);
            Util.writeToCSV("./multipleSclerosisOdel.csv",l_values);
            Util.writeToCSV("./multipleSclerosisOdeL.csv",L_values);


        } catch (RuntimeException e) {
            e.printStackTrace();
        }



    }

    public static DataState getInitialState() {
        Map<Integer, Double> values = new HashMap<>();

        values.put(E, Einit);
        values.put(Er, 0.0);
        values.put(R, Rinit);
        values.put(Rr, 0.0);
        values.put(Ea, Math.pow(Einit/a,2));
        values.put(l, 0.0);
        values.put(L, 0.0);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }


    public static List<DataStateUpdate> odeEnv(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(Er,state.get(Er) + ( - state.get(Er)*delta - state.get(Er)*beta + state.get(E)*eta ) ) );
        updates.add(new DataStateUpdate(Rr,state.get(Rr) + ( - state.get(Rr)*delta - state.get(Rr)*beta - state.get(R)*eta ) ) );
        updates.add(new DataStateUpdate(E, state.get(E) + ( state.get(Er)*delta - state.get(E) * eta + state.get(E)*alphaE*Math.pow(kR,h)/(Math.pow(kR,h)+Math.pow(state.get(R),h)) - state.get(E)*gammaE*Math.pow(R,h)/(Math.pow(kR,h)+Math.pow(state.get(R),h))) ) );
        updates.add(new DataStateUpdate(R, state.get(R) + ( state.get(Rr)*delta - state.get(R) * eta + state.get(R)*alphaR*Math.pow(E,h)/(Math.pow(kE,h)+Math.pow(state.get(E),h)) - state.get(R)*gammaE) ) );
        return updates;

    }



}
