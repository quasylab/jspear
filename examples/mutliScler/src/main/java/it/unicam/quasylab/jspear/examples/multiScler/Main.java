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

package it.unicam.quasylab.jspear.examples.multiScler;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.*;
import it.unicam.quasylab.jspear.distance.*;
import it.unicam.quasylab.jspear.distl.*;
import it.unicam.quasylab.jspear.ds.*;
import it.unicam.quasylab.jspear.perturbation.AtomicPerturbation;
import it.unicam.quasylab.jspear.perturbation.IterativePerturbation;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.perturbation.SequentialPerturbation;
import it.unicam.quasylab.jspear.robtl.*;
import nl.tue.Monitoring.Default.DefaultMonitorBuilder;
import nl.tue.Monitoring.Default.DefaultUDisTLMonitor;
import nl.tue.Monitoring.PerceivedSystemState;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.stat.ranking.NaNStrategy;

import java.io.IOException;
import java.math.BigDecimal;
import java.util.*;

public class Main {

    public static final int E = 0; // active Teff (Effector T cells)
    public static final int Er = 1; // resting Teff (Effector T cells)
    public static final int R = 2; // active Treg (Regulatory T cells)
    public static final int Rr = 3; // resting Treg (Regulatory T cells)
    public static final int Ea = 4; // threshold used in the second order effect of Teff cells on damage
    public static final int l = 5; // reversible damage
    public static final int L = 6; // irreversible damage
    public static final int ratioER = 7;
    public static final int v_eta = 8; // cell rest rate
    public static final int v_delta = 9; // cell activation rate
    public static final int v_beta = 10; // resting cell death rate
    public static final int v_gammaE = 11; // active Teff death rate
    public static final int v_gammaR = 12; // active Treg death rate
    public static final int v_d1 = 13; // reversible damage rate
    public static final int v_d2 = 14; // irreversible damage rate
    public static final int v_r = 15; // recovery rate
    public static final int alphaR = 16; // Treg proliferation rate
    public static final int timer = 17;
    public static final int uncertainty = 18;

    private static final int NUMBER_OF_VARIABLES = 19;


    public static final double eta = 0.01; // initial value for v_eta
    public static final double delta = 1.0; // initial value for v_delta
    public static final double beta = 0.01; // initial value for v_beta
    /*
    public static final double k1 = 1.0;
    public static final double k2 = 0.25;
    public static final double k3 = 0.1;
     */
    public static final double alphaE = 2.0; // initial value for v_alphaE
    public static final double alphaRH = 1.0; // healthy value for v_alphaR
    public static final double alphaRS = 0.25; // unhealthy value for v_alphaR
    public static final double gammaE = 0.2; // initial value for v_gammaE
    public static final double gammaR = 0.2; // initial value for v_gammaR
    public static final double kE = 1000.0; // Hill dynamic Teff coefficient
    public static final double kR = 200.0; // Hill dynamic Treg coefficient
    /* public static final double km1 = k1*Math.pow(kR,2) - gammaE;
    public static final double km2 = k2*kE - alphaR;
    public static final double km3 = k3*Math.pow(kR,2) - alphaE;
     */
    public static final double d1 = 1.0; // initial value for v_d1
    public static final double d2 = 0.02; // initial value for v_d2
    public static final double r = 0.1; // initial value for v_r
    private static final double a = 22800.0; // initial value for Ea
    private static final double Einit = 1000.0; // initial value for Teff
    private static final double Rinit = 200.0; // initial value for Treg
    private static final double h = 5.0; // hill coefficient
    private static final double delta_t = Math.pow(10,-4); //
    private static final double jumps = 1;

    public static void main(String[] args) throws IOException {
        try {

            Controller controller = getController();

            /*
            Two systems are created, one is healthy (alphaR = alphaRH = 1.0), the other is unhealthy (alphaR = alphaRH = 0.25)
             */
            DataState stateH = getInitialState(jumps, 0.0, 0.0, delta_t, alphaRH, 1.0);
            DataState stateS = getInitialState(jumps, 0.0, 0.0, delta_t, alphaRS, 1.0);
            RandomGenerator rand = new DefaultRandomGenerator();
            TimedSystem systemH1 = new TimedSystem(controller, (rg, ds) -> ds.apply(odeEnv(rg, ds,1)), stateH, ds -> ds.getTimeDelta());
            TimedSystem systemH5 = new TimedSystem(controller, (rg, ds) -> ds.apply(odeEnv(rg, ds,5)), stateH, ds -> ds.getTimeDelta());
            TimedSystem systemS1 = new TimedSystem(controller, (rg, ds) -> ds.apply(odeEnv(rg, ds,1)), stateS, ds -> ds.getTimeDelta());
            TimedSystem systemS5 = new TimedSystem(controller, (rg, ds) -> ds.apply(odeEnv(rg, ds,5)), stateS, ds -> ds.getTimeDelta());

            int size = 100;
            int size_sim = 100;
            int steps = 2000;




            /*
            The unhealthy system is simulated in order to estimate:
             1) the worst (i.e. maximal) ratio Teff/Treg exhibited in size_sim executions that run for 2000 days.
             Rationale: A high Teff/Treg ratio is responsible for the relapses of the disease.
             2) the worst (i.e. maximal) level of the reversible damage exhibited in size_sim executions that run for 2000 days.
             Rationale: A high level of the reversible damage impacts negatively on the short term quality of life.
             The healthy system is also simulated and the worst Teff/Treg ratio and reversible damage value exhibited by both systems are printed out.
             */
            double maxRatioERH = 0.0;
            double maxRatioERS = 0.0;
            double maxRevDamageH = 0.0;
            double maxRevDamageS = 0.0;

            ArrayList<DataStateExpression> Fworst = new ArrayList<>();
            Fworst.add(ds -> ds.get(E) / ds.get(R));
            Fworst.add(ds -> ds.get(l));




            double[][] ratio_E_R_rev_dam_max_H = SystemState.sample_max(rand, Fworst, systemH1, steps, size_sim);
            for (int i = 0; i < ratio_E_R_rev_dam_max_H.length; i++) {
                if (ratio_E_R_rev_dam_max_H[i][0] > maxRatioERH) {
                    maxRatioERH = ratio_E_R_rev_dam_max_H[i][0];
                }
                if (ratio_E_R_rev_dam_max_H[i][1] > maxRevDamageH) {
                    maxRevDamageH = ratio_E_R_rev_dam_max_H[i][1];
                }
            }
            Util.writeToCSV("./multipleSclerosisOdeRatioERHealthy.csv", ratio_E_R_rev_dam_max_H);

            double[][] ratio_E_R_rev_dam_max_S = SystemState.sample_max(rand, Fworst, systemS1, steps, size_sim);
            for (int i = 0; i < ratio_E_R_rev_dam_max_S.length; i++) {
                if (ratio_E_R_rev_dam_max_S[i][0] > maxRatioERS) {
                    maxRatioERS = ratio_E_R_rev_dam_max_S[i][0];
                }
                if (ratio_E_R_rev_dam_max_S[i][1] > maxRevDamageS) {
                    maxRevDamageS = ratio_E_R_rev_dam_max_S[i][1];
                }
            }
            Util.writeToCSV("./multipleSclerosisOdeRatioERSick.csv", ratio_E_R_rev_dam_max_S);

            System.out.println(" ");
            System.out.println(" ");
            System.out.println("Maximal Eff/Reg ratio exhibited by the healthy system in " + size_sim + " runs: " + maxRatioERH);
            System.out.println(" ");
            System.out.println(" ");
            System.out.println("Maximal Eff/Reg ratio exhibited by the unhealthy system in " + size_sim + " runs: " + maxRatioERS);
            System.out.println(" ");
            System.out.println(" ");
            System.out.println("Maximal rev. damage exhibited by the healthy system in " + size_sim + " runs: " + maxRevDamageH);
            System.out.println(" ");
            System.out.println(" ");
            System.out.println("Maximal rev. damage exhibited by the unhealthy system in " + size_sim + " runs: " + maxRevDamageS);
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");



            final double maxRatioERHF = maxRatioERH;
            final double maxRatioERSF = maxRatioERS;
            final double maxRevDamageHF = maxRevDamageH;
            final double maxRevDamageSF = maxRevDamageS;




            /*
            In order to observe the behaviour of systems, both the healthy and the unhealthy systems are simulated.
            For each system we take size executions that run for 2000 days.
            We print out the average value over the size runs that is taken, day per day, by some variables.
            These values are also registered in a csv file for plotting.
             */

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds -> ds.get(E));
            F.add(ds -> ds.get(R));
            F.add(ds -> ds.get(Er));
            F.add(ds -> ds.get(Rr));
            F.add(ds -> ds.get(l));
            F.add(ds -> ds.get(L));
            F.add(ds -> (ds.get(E) / ds.get(R)));

            double[][] data_avgH = SystemState.sample(rand, F, systemH5, steps, size);
            double[][] data_avgS = SystemState.sample(rand, F, systemS5, steps, size);


            System.out.println("Healthy systems: Average value for E, R, Er, Rr, l, L, E/R in " +size+ " runs "+ steps +" days ");
            for (int i = 0; i < data_avgH.length; i++) {
                System.out.printf("%d>   ", i);
                for (int j = 0; j < data_avgH[i].length - 1; j++) {
                    System.out.printf("%f   ", data_avgH[i][j]);
                }
                System.out.printf("%f\n", data_avgH[i][data_avgH[i].length - 1]);
            }

            System.out.println(" ");
            System.out.println(" ");
            System.out.println("Unhealthy systems: Average value for E, R, Er, Rr, l, L, E/R in " +size+ " runs "+ steps +" days ");
            System.out.println(" ");

            for (int i = 0; i < data_avgS.length; i++) {
                System.out.printf("%d>   ", i);
                for (int j = 0; j < data_avgS[i].length - 1; j++) {
                    System.out.printf("%f   ", data_avgS[i][j]);
                }
                System.out.printf("%f\n", data_avgS[i][data_avgS[i].length - 1]);
            }

            Util.writeToCSV("./multipleSclerosisOdeHealthy.csv", data_avgH);
            Util.writeToCSV("./multipleSclerosisOdeSick.csv", data_avgS);



            /*
            An evolution sequence of the healthy system is generated in order to generate distributions of configurations that can be used as "targets"
             */

            EvolutionSequence healthySeq = new EvolutionSequence(new DefaultRandomGenerator(), rg -> systemH1, size);

            /*
            The DisTL formula ratioEffRegBounded is created.
            When evaluated on an evolution sequence S at a given step n, the formula evaluates how much the distribution reached by S at day n is close to a healthy distribution with respect to the value of the Eff/Reg ratio.
             */

            SampleSet<SystemState> targetStateDistr = healthySeq.get(100);
            DataStateExpression rho_eff_reg_ratio = ds -> Math.min(1.0, Math.max(0.0, ds.get(E) / ds.get(R) - maxRatioERHF) / maxRatioERSF);
            DisTLFormula ratioEffRegBounded = new TargetDisTLFormula(targetStateDistr, rho_eff_reg_ratio, 0.1);

            /*
            The DisTL formula revDamageBounded is created.
            When evaluated on an evolution sequence S at a given step n, the formula evaluates how much the distribution reached by S at day n is close to a healthy distribution with respect to the value of the reversible damage.
             */
            DataStateExpression rho_rev_dam = ds -> Math.min(1.0, Math.max(0.0, ds.get(l) - maxRevDamageHF) / maxRevDamageSF);
            DisTLFormula revDamageBounded = new TargetDisTLFormula(targetStateDistr, rho_rev_dam, 0.1);

            /*
            The DisTL formulas alwaysRatioEffRegBounded / alwaysRevDamageBounded are the universal quantification over an interval of ratioEffRegBounded and revDamageBounded, respectively.
            */
            DisTLFormula alwaysRatioEffRegBounded = new AlwaysDisTLFormula(ratioEffRegBounded, 0, 1999);
            DisTLFormula alwaysRevDamageBounded = new AlwaysDisTLFormula(revDamageBounded, 0, 1999);


            /*
            The DisTL formula recoveryRevDamage expresses the ability to remit in a suitable time interval
             */
            DisTLFormula recoveryRevDamage = new EventuallyDisTLFormula(revDamageBounded,5,10);

            /*
            The DisTL formula recoveryAfterRatioEffRegHigh expresses the ability to remit after a relapsing phase.
            The DisTL formula alwaysRecoveryAfterRatioEffRegHigh is the universal quantification over an interval of recoveryAfterRatioEffRegHigh.
             */
            DisTLFormula recoveryAfterRatioEffRegHigh = new ImplicationDisTLFormula(new NegationDisTLFormula(ratioEffRegBounded),recoveryRevDamage);
            DisTLFormula alwaysRecoveryAfterRatioEffRegHigh = new AlwaysDisTLFormula(recoveryAfterRatioEffRegHigh,0,1999);


            EvolutionSequence hSeq = new EvolutionSequence(new DefaultRandomGenerator(), rg -> systemH5, 10);
            EvolutionSequence sSeq = new EvolutionSequence(new DefaultRandomGenerator(), rg -> systemS5, 10);

            double v1 = new DoubleSemanticsVisitor().eval(alwaysRatioEffRegBounded).eval(1, 0, hSeq);
            double v2 = new DoubleSemanticsVisitor().eval(alwaysRatioEffRegBounded).eval(1, 0, sSeq);
            double v3 = new DoubleSemanticsVisitor().eval(alwaysRevDamageBounded).eval(1, 0, hSeq);
            double v4 = new DoubleSemanticsVisitor().eval(alwaysRevDamageBounded).eval(1, 0, sSeq);
            double v5 = new DoubleSemanticsVisitor().eval(alwaysRecoveryAfterRatioEffRegHigh).eval(1, 0, hSeq);
            double v6 = new DoubleSemanticsVisitor().eval(alwaysRecoveryAfterRatioEffRegHigh).eval(1, 0, sSeq);



            System.out.println("evaluation of alwaysRatioEffRegBounded, healthy system = " + v1);
            System.out.println("evaluation of alwaysRatioEffRegBounded, unhealthy system = " + v2);
            System.out.println("evaluation of alwaysRevDamageBounded, healthy system = " + v3);
            System.out.println("evaluation of alwaysRevDamageBounded, unhealthy system = " + v4);
            System.out.println("evaluation of alwaysRecoveryAfterRatioEffRegHigh, healthy system)= " + v5);
            System.out.println("evaluation of alwaysRecoveryAfterRatioEffRegHigh, unhealthy system = " + v6);


            /*
            for(int i=0;i<100;i++){System.out.println(" ");}


            DisTLFormula ff = new AlwaysDisTLFormula(new TargetDisTLFormula(targetStateDistr, rho_eff_reg_ratio, 0.4), 100, 2000);


            EvolutionSequence hSeqq = new EvolutionSequence(new DefaultRandomGenerator(), lrg -> systemH, 10);
            EvolutionSequence sSeqq = new EvolutionSequence(new DefaultRandomGenerator(), rg -> systemS, 10);


            DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(10, false);

            DefaultUDisTLMonitor m = defaultMonitorBuilder.build(ff);
            DefaultUDisTLMonitor m2 = defaultMonitorBuilder.build(ff);

            int i = 0;
            while (i < 2000) {
                SampleSet<PerceivedSystemState> distribution = sSeqq.getAsPerceivedSystemStates(i);
                OptionalDouble monitorEval = m.evalNext(distribution);
                System.out.println(monitorEval.isPresent() ? monitorEval.getAsDouble() : "u");

                i++;
            }


            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");



             */

            DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(size, false);

            DefaultUDisTLMonitor monitorEffReg = defaultMonitorBuilder.build(alwaysRatioEffRegBounded);
            DefaultUDisTLMonitor monitorRevDam = defaultMonitorBuilder.build(alwaysRevDamageBounded);
            DefaultUDisTLMonitor monitorRecovery = defaultMonitorBuilder.build(alwaysRecoveryAfterRatioEffRegHigh);


            double[][] resultsOfMonitoring = new double[2000][3];

            System.out.println("monitoring of alwaysRatioEffRegBounded, unhealthy system");
            int i = 0;
            while (i < 2000) {
                SampleSet<PerceivedSystemState> distribution = sSeq.getAsPerceivedSystemStates(i);
                OptionalDouble monitorEval = monitorEffReg.evalNext(distribution);
                System.out.println(monitorEval.isPresent() ? monitorEval.getAsDouble() : "u");
                resultsOfMonitoring[i][0] = monitorEval.isPresent() ? monitorEval.getAsDouble() : 0;
                i++;
            }

            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");




            System.out.println("monitoring of alwaysRevDamageBounded, unhealthy system");
            i = 0;
            while (i < 2000) {
                SampleSet<PerceivedSystemState> distribution = sSeq.getAsPerceivedSystemStates(i);
                OptionalDouble monitorEval = monitorRevDam.evalNext(distribution);
                System.out.println(monitorEval.isPresent() ? monitorEval.getAsDouble() : "u");
                resultsOfMonitoring[i][1] = monitorEval.isPresent() ? monitorEval.getAsDouble() : 0;
                i++;
            }


            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");
            System.out.println(" ");


            System.out.println("monitoring of alwaysRecoveryAfterRatioEffRegHigh, unhealthy system");
            i = 0;
            while (i < 2000) {
                SampleSet<PerceivedSystemState> distribution = sSeq.getAsPerceivedSystemStates(i);
                OptionalDouble monitorEval = monitorRecovery.evalNext(distribution);
                System.out.println(monitorEval.isPresent() ? monitorEval.getAsDouble() : "u");
                resultsOfMonitoring[i][2] = monitorEval.isPresent() ? monitorEval.getAsDouble() : 0;
                i++;
            }

            Util.writeToCSV("./multipleSclerosisOdeMonitoredValues.csv", resultsOfMonitoring);

            







            } catch(RuntimeException e){
                e.printStackTrace();
            }

        }

    public static Controller getController() {
        return new NilController();
    }







    public static DataState getInitialState(double gran, double Tstep, double Ttot, double Tshift, double alphaR_value, double uncertainty_value) {
        Map<Integer, Double> values = new HashMap<>();

        values.put(E, Einit);
        values.put(Er, 0.0);
        values.put(R, Rinit);
        values.put(Rr, 0.0);
        values.put(Ea, Math.pow(Einit/a,2));
        values.put(l, 0.0);
        values.put(L, 0.0);
        values.put(ratioER, Einit / Rinit);
        values.put(v_eta,eta);
        values.put(v_delta,delta);
        values.put(v_beta,beta);
        values.put(v_gammaE,gammaE);
        values.put(v_gammaR,gammaR);
        values.put(v_d1,d1);
        values.put(v_d2,d2);
        values.put(v_r,r);
        values.put(timer, 0.0);
        values.put(alphaR,alphaR_value);
        values.put(uncertainty,uncertainty_value);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Ttot, Tshift);
    }


    public static List<DataStateUpdate> odeEnv(RandomGenerator rg, DataState state, int var) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double old_Er = state.get(Er);
        double old_E = state.get(E);
        double old_Rr = state.get(Rr);
        double old_R = state.get(R);
        double old_l = state.get(l);
        double old_L = state.get(L);
        double old_timer = state.get(timer);
        double old_eta = state.get(v_eta);
        double old_delta = state.get(v_delta);
        double old_beta = state.get(v_beta);
        double old_gammaE = state.get(v_gammaE);
        double old_gammaR = state.get(v_gammaR);
        double old_d1 = state.get(v_d1);
        double old_d2 = state.get(v_d2);
        double old_r = state.get(v_r);
        double old_alphaR = state.get(alphaR);


        double r1 = rg.nextDouble();
        double ie;
        if (r1<100*delta_t/365.0) {ie=100.0/delta_t;} else{ie = 0;}
        double ir;
        double r2 = rg.nextDouble();
        if (r2<100*delta_t/365.0) {ir=100.0/delta_t;} else{ir = 0;}

        double dEr = ie - old_Er*old_delta - old_Er*old_beta + old_E*old_eta;
        double new_Er = old_Er + dEr*delta_t;
        updates.add(new DataStateUpdate(Er, new_Er));
        double dRr = ir - old_Rr*old_delta - old_Rr*old_beta + old_R*old_eta;
        double new_Rr = old_Rr + dRr*delta_t;
        updates.add(new DataStateUpdate(Rr, new_Rr));
        double dE = old_Er*old_delta - old_E*old_eta + old_E*(alphaE*Math.pow(kR,h) - old_gammaE*Math.pow(old_R,h))/(Math.pow(kR,h)+Math.pow(old_R,h));
        double new_E = old_E + dE*delta_t;
        updates.add(new DataStateUpdate(E, new_E));
        double dR = old_Rr*old_delta - old_R*old_eta + old_R*old_alphaR*Math.pow(old_E,h)/(Math.pow(kE,h)+Math.pow(old_E,h)) - old_R*old_gammaR;
        double new_R = old_R + dR*delta_t;
        updates.add(new DataStateUpdate(R, new_R));

        /* double new_Ea = Math.pow(new_E/a,2); */
        double new_Ea = Math.pow(old_E/a,2);
        updates.add(new DataStateUpdate(Ea, new_Ea));
        double dl = new_Ea*old_d1 - old_l*old_r - old_l*old_d2;
        double new_l = old_l + dl*delta_t;
        updates.add(new DataStateUpdate(l,new_l));
        double dL = old_l*old_d2;
        double new_L = old_L + dL*delta_t;
        updates.add(new DataStateUpdate(L, new_L));

        updates.add(new DataStateUpdate(ratioER, new_E/new_R));

        if(old_timer>=1){
            updates.add(new DataStateUpdate(timer, 0.0));

            double r3 = rg.nextDouble()*var*2;
            double e_eta = (r3-var)*old_eta/100.0;
            updates.add(new DataStateUpdate(v_eta,old_eta+e_eta));

            double r4 = rg.nextDouble()*var*2;
            double e_delta = (r4-var)*old_delta/100.0;
            updates.add(new DataStateUpdate(v_delta,old_delta+e_delta));

            double r5 = rg.nextDouble()*var*2;
            double e_beta = (r5-var)*old_beta/100.0;
            updates.add(new DataStateUpdate(v_beta,old_beta+e_beta));

            //r3 = rg.nextDouble()*var*2;
            //double e_gammaE = (r3-var)*old_gammaE/100.0;
            //updates.add(new DataStateUpdate(v_gammaE,old_gammaE+e_gammaE));

            //r3 = rg.nextDouble()*var*2;
            //double e_gammaR = (r3-var)*old_gammaR/100.0;
            //updates.add(new DataStateUpdate(v_gammaR,old_gammaR+e_gammaR));

            double r6 = rg.nextDouble()*var*2;
            double e_d1 = (r6-var)*old_d1/100.0;
            updates.add(new DataStateUpdate(v_d1,old_d1+e_d1));

            double r7 = rg.nextDouble()*var*2;
            double e_d2 = (r7-var)*old_d2/100.0;
            updates.add(new DataStateUpdate(v_d2,old_d2+e_d2));

            double r8 = rg.nextDouble()*var*2;
            double e_r = (r8-var)*old_r/100.0;
            updates.add(new DataStateUpdate(v_r,old_r+e_r));
        }
        else{
            updates.add(new DataStateUpdate(timer, old_timer + delta_t));
        }

        return updates;

    }



}