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
import it.unicam.quasylab.jspear.controller.*;
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

    public static final int E1 = 7;
    public static final int E2 = 8;
    public static final int E3 = 9;
    public static final int E4 = 10;
    public static final int E5 = 11;
    public static final int E6 = 12;
    public static final int E7 = 13;
    public static final int E8 = 14;
    public static final int E9 = 15;
    public static final int E10 = 16;

    public static final int R1 = 17;
    public static final int R2 = 18;
    public static final int R3 = 19;
    public static final int R4 = 20;
    public static final int R5 = 21;
    public static final int R6 = 22;
    public static final int R7 = 23;
    public static final int R8 = 24;
    public static final int R9 = 25;
    public static final int R10 = 26;



    private static final int NUMBER_OF_VARIABLES = 27;

    public static final double IE = 0.0;
    public static final double IR = 0.0;
    public static final double eta = 0.01;
    public static final double delta = 1.0;
    public static final double beta = 0.01;
    public static final double k1 = 1.0;
    public static final double k2 = 0.25;
    public static final double k3 = 0.1;
    public static final double alphaE = 2.0;
    public static final double alphaR = 0.25;
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
    private static final double delta_t = Math.pow(10,-4);
    private static final double jumps = 1;

    public static void main(String[] args) throws IOException {
        try {

            Controller controller = getController();

            DataState state = getInitialState(jumps,0.0,0.0,delta_t);

            RandomGenerator rand = new DefaultRandomGenerator();

            TimedSystem system = new TimedSystem(controller, (rg, ds) -> ds.apply(odeEnv(rg, ds)), state, ds -> ds.getTimeDelta());

            int size = 1;

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(E));
            F.add(ds->ds.get(R));
            F.add(ds->ds.get(Er));
            F.add(ds->ds.get(Rr));
            F.add(ds->ds.get(l));
            F.add(ds->ds.get(L));
            F.add(ds ->(ds.get(E)/ds.get(R)));

            int steps = 2000;


            double[][] data_avg = SystemState.sample(rand, F, system, steps, size);
            double[][] E_values = new double[steps][1];
            double[][] R_values = new double[steps][1];
            double[][] Er_values = new double[steps][1];
            double[][] Rr_values = new double[steps][1];
            double[][] l_values= new double[steps][1];
            double[][] L_values = new double[steps][1];
            double[][] E_R_values = new double[steps][1];



            for (int i = 0; i < data_avg.length; i++) {
                System.out.printf("%d>   ", i);
                for (int j = 0; j < data_avg[i].length -1 ; j++) {
                    System.out.printf("%f   ", data_avg[i][j]);
                }
                System.out.printf("%f\n", data_avg[i][data_avg[i].length -1]);
            }

            for(int j = 0; j < steps; j++){
                E_values[j][0] = data_avg[j][0];
                R_values[j][0] = data_avg[j][1];
                Er_values[j][0] = data_avg[j][2];
                Rr_values[j][0] = data_avg[j][3];
                l_values[j][0] = data_avg[j][4];
                L_values[j][0] = data_avg[j][5];
                E_R_values[j][0] = data_avg[j][0]/data_avg[j][1];
            }

            Util.writeToCSV("./multipleSclerosisOde_sick.csv",data_avg);
            Util.writeToCSV("./multipleSclerosisOdeE.csv",E_values);
            Util.writeToCSV("./multipleSclerosisOdeR.csv",R_values);
            Util.writeToCSV("./multipleSclerosisOdeEr.csv",Er_values);
            Util.writeToCSV("./multipleSclerosisOdeRr.csv",Rr_values);
            Util.writeToCSV("./multipleSclerosisOdel.csv",l_values);
            Util.writeToCSV("./multipleSclerosisOdeLLL.csv",L_values);


        } catch (RuntimeException e) {
            e.printStackTrace();
        }

    }

    public static Controller getController() {
        return new NilController();
    }


    public static Controller getController(int x) {
        ControllerRegistry registry = getControllerRegistry();
        return registry.reference("therapy");
    }


    public static ControllerRegistry getControllerRegistry() {
        ControllerRegistry registry = new ControllerRegistry();

        registry.set("therapy",
                Controller.ifThenElse(
                        DataState.greaterThan(R1,R2).and(DataState.greaterThan(R2,R3).and(DataState.greaterThan(R3,R4).and(DataState.greaterThan(R4,R5).and(DataState.greaterThan(R5,R6).and(DataState.greaterThan(R6,R7).and(DataState.greaterThan(R7,R8).and(DataState.greaterThan(R8,R9).and(DataState.greaterThan(R9,R10).and(DataState.greaterThan(E2,E1).and(DataState.greaterThan(E3,E2).and(DataState.greaterThan(E4,E3).and(DataState.greaterThan(E5,E4).and(DataState.greaterThan(E6,E5).and(DataState.greaterThan(E7,E6).and(DataState.greaterThan(E8,E7).and(DataState.greaterThan(E9,E8).and(DataState.greaterThan(E10,E9)))))))))))))))))),
                        Controller.doAction((rg,ds)-> List.of(new DataStateUpdate(R,ds.get(R)*1.0)),registry.reference("therapy")),
                        Controller.doTick(registry.reference("therapy"))
                ));

        return registry;
    }


    public static DataState getInitialState(double gran, double Tstep, double Ttot, double Tshift) {
        Map<Integer, Double> values = new HashMap<>();

        values.put(E, Einit);
        values.put(Er, 0.0);
        values.put(R, Rinit);
        values.put(Rr, 0.0);
        values.put(Ea, Math.pow(Einit/a,2));
        values.put(l, 0.0);
        values.put(L, 0.0);
        values.put(E1,Einit);
        values.put(E2,Einit);
        values.put(E3,Einit);
        values.put(E4,Einit);
        values.put(E5,Einit);
        values.put(E6,Einit);
        values.put(E7,Einit);
        values.put(E8,Einit);
        values.put(E9,Einit);
        values.put(E10,Einit);
        values.put(R1,Rinit);
        values.put(R2,Rinit);
        values.put(R3,Rinit);
        values.put(R4,Rinit);
        values.put(R5,Rinit);
        values.put(R6,Rinit);
        values.put(R7,Rinit);
        values.put(R8,Rinit);
        values.put(R9,Rinit);
        values.put(R10,Rinit);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Ttot, Tshift);
    }


    public static List<DataStateUpdate> odeEnv(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double old_Er = state.get(Er);
        double old_E = state.get(E);
        double old_Rr = state.get(Rr);
        double old_R = state.get(R);
        double old_l = state.get(l);
        double old_L = state.get(L);


        double r1 = rg.nextDouble();
        double ie;
        if (r1<100*delta_t/365.0) {ie=100.0/delta_t;} else{ie = 0;}
        double ir;
        double r2 = rg.nextDouble();
        if (r2<100*delta_t/365.0) {ir=100.0/delta_t;} else{ir = 0;}

        double dEr = ie - old_Er*delta - old_Er*beta + old_E*eta;
        double new_Er = old_Er + dEr*delta_t;
        updates.add(new DataStateUpdate(Er, new_Er));
        double dRr = ir - old_Rr*delta - old_Rr*beta + old_R*eta;
        double new_Rr = old_Rr + dRr*delta_t;
        updates.add(new DataStateUpdate(Rr, new_Rr));
        double dE = old_Er*delta - old_E*eta + old_E*(alphaE*Math.pow(kR,h) - gammaE*Math.pow(old_R,h))/(Math.pow(kR,h)+Math.pow(old_R,h));
        double new_E = old_E + dE*delta_t;
        updates.add(new DataStateUpdate(E, new_E));
        double dR = old_Rr*delta - old_R*eta + old_R*alphaR*Math.pow(old_E,h)/(Math.pow(kE,h)+Math.pow(old_E,h)) - old_R*gammaR;
        double new_R = old_R + dR*delta_t;
        updates.add(new DataStateUpdate(R, new_R));

        /* double new_Ea = Math.pow(new_E/a,2); */
        double new_Ea = Math.pow(old_E/a,2);
        updates.add(new DataStateUpdate(Ea, new_Ea));
        double dl = new_Ea*d1 - old_l*r - old_l*d2;
        double new_l = old_l + dl*delta_t;
        updates.add(new DataStateUpdate(l,new_l));
        double dL = old_l*d2;
        double new_L = old_L + dL*delta_t;
        updates.add(new DataStateUpdate(L, new_L));

        return updates;

    }



}
