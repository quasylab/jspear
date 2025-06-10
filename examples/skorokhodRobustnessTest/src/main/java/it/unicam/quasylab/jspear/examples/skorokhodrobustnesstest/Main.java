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

package it.unicam.quasylab.jspear.examples.skorokhodrobustnesstest;



import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.random.RandomGenerator;

import it.unicam.quasylab.jspear.DefaultRandomGenerator;
import it.unicam.quasylab.jspear.EvolutionSequence;
import it.unicam.quasylab.jspear.TimedSystem;
import it.unicam.quasylab.jspear.Util;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.NilController;
import it.unicam.quasylab.jspear.distance.SkorokhodDistanceExpression;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;

public class Main {

    /*
    THE TWO STATE MODEL OF THE REPRESSILATOR

    The Skorokhod distance expression can allow translations between perturbed and nominal sequences.
    This Program will introduce a nominal sinusoid, and a perturbed sinusoid whose wave is shifted.
    The distance expression should find the distance and return a value of 0.
     */

    public static final double x_init = 0.0;
    public static final int x_key = 0;

    public static final double transFrom = 250;
    public static final double transPert = 30.0;
    public static final double freqPert = 2.0;
    public static final double frequency = (1.0 / 20.0);

    // MAIN PROGRAM
    public static void main(String[] args) throws IOException {
        try {

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
             */
            RandomGenerator rand = new DefaultRandomGenerator();

            TimedSystem system = new TimedSystem(controller, (rg, ds) -> ds.apply(updateState(rg, ds)), state, ds->selectReactionTime(rand,ds));

            TimedSystem systemTransPerturbed = new TimedSystem(controller, (rg, ds) -> ds.apply(getPerturbedTranslation(rg, ds)), state, ds->selectReactionTime(rand,ds));

            TimedSystem systemFreqPerturbed = new TimedSystem(controller, (rg, ds) -> ds.apply(getPerturbedFrequency(rg, ds)), state, ds->selectReactionTime(rand,ds));

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

            Since this test program is deterministic, a size of 1 suffices.
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

            EvolutionSequence sequenceTransPerturbed = new EvolutionSequence(rand, rg -> systemTransPerturbed, size);

            EvolutionSequence sequenceFreqPerturbed = new EvolutionSequence(rand, rg -> systemFreqPerturbed, size);

            /*
            Each expression in the following list <code>F</code> allows us to read the value of a given variable
            from a data state
             */
            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds->ds.get(x_key));

            ArrayList<String> L = new ArrayList<>();
            L.add("x      ");

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

            /*
            The following instruction allows us to create the evolution sequence <code>sequence_p</code>, which is
            obtained from the evolution sequence <code>sequence</code> by applying a perturbation, where:
            - as above, the perturbation is returned by the static method <code>itZ1PertRate()</code> defined later
            - the perturbation is applied at step 0
            - the sample sets of configurations in <code>sequence_p</code> have a cardinality which corresponds to that
            of <code>sequence</code> multiplied by <code>scale>/code>
            */

            int N = 500;    // length ot the evolution sequence
            int leftBound = 0;
            int rightBound = 500;
            int normalisationTime = 1000;
            int scanWidth = 300;
            int scanFromStep = 0;
            int lambdaCount = 200;
            
            SkorokhodDistanceExpression skorokhod = new SkorokhodDistanceExpression(ds->ds.get(x_key),
                                                                                        (v1, v2) -> Math.abs(v2-v1),
                                                                                        (a, b) -> Math.max(a, b),
                                                                                        offset->((double)offset/(double)normalisationTime),
                                                                                        scanFromStep,
                                                                                        rightBound,true, lambdaCount, scanWidth);

            double[][] direct_evaluation_Z1 = new double[rightBound-leftBound][1];
            double[][] plotx = new double[N][1];
            double[][] pplotx = new double[N][1];
            for (int i = 0; i<(rightBound-leftBound); i++){
                direct_evaluation_Z1[i][0] = skorokhod.compute(i+leftBound, sequence, sequenceTransPerturbed);
                plotx[i][0] = sequence.get(i).evalPenaltyFunction(ds->ds.get(x_key))[0];
                pplotx[i][0] = sequenceTransPerturbed.get(i).evalPenaltyFunction(ds->ds.get(x_key))[0];
            }
            Util.writeToCSV("./distance.csv",direct_evaluation_Z1);
            Util.writeToCSV("./plotx.csv",plotx);
            Util.writeToCSV("./pplotx.csv",pplotx);

            int[] _offsets = skorokhod.GetOffsetArray();
            double[][] offsets = new double[rightBound][1];
            for (int i = 0; i < offsets.length; i++) {
                offsets[i][0] = _offsets[i];
            }

            Util.writeToCSV("./offsets.csv",offsets);

            // int offsetEvaluationCount2 = 200;
            // int scanWidth2 = 200;

            // run again, comparing to high frequency:
            SkorokhodDistanceExpression skorokhod2 = new SkorokhodDistanceExpression(ds->ds.get(x_key),
                                                                                        (v1, v2) -> Math.abs(v2-v1),
                                                                                        (a, b) -> Math.max(a, b),
                                                                                        offset->((double)offset/(double)normalisationTime),
                                                                                        scanFromStep,
                                                                                        rightBound,true, lambdaCount, scanWidth);

            for (int i = 0; i<(rightBound-leftBound); i++){
                direct_evaluation_Z1[i][0] = skorokhod2.compute(i+leftBound, sequence, sequenceFreqPerturbed);
                pplotx[i][0] = sequenceFreqPerturbed.get(i).evalPenaltyFunction(ds->ds.get(x_key))[0];
            }
            Util.writeToCSV("./distanceHF.csv",direct_evaluation_Z1);
            Util.writeToCSV("./pplotxHF.csv",pplotx);

            _offsets = skorokhod2.GetOffsetArray();
            for (int i = 0; i < offsets.length; i++) {
                offsets[i][0] = _offsets[i];
            }
            Util.writeToCSV("./offsetsHF.csv",offsets);

        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }

    // select time after which new state update will be:
    public static double selectReactionTime(RandomGenerator rg, DataState state){
        return 0.1;
    }

    // get change in state over time:
    public static List<DataStateUpdate> updateState(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        updates.add(new DataStateUpdate(x_key, Math.sin(frequency * state.getTimeReal())));

        return updates;
    }
    
    // get change in state over time:
    public static List<DataStateUpdate> getPerturbedTranslation(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        
        if (state.getTimeReal() >= transFrom)
        {
            updates.add(new DataStateUpdate(x_key, Math.sin(frequency * (state.getTimeReal() - transPert))));
        }
        else
        {
            // return nominal state
            return updateState(rg, state);
        }
        
        return updates;
    }

    // get change in state over time:
    public static List<DataStateUpdate> getPerturbedFrequency(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        if (state.getTimeReal() >= transFrom)
        {
            updates.add(new DataStateUpdate(x_key, Math.sin(frequency * (state.getTimeReal() * freqPert))));
        }
        else
        {
            // return nominal state
            return updateState(rg, state);
        }
        return updates;
    }

    /*
    Method getInitialState assigns the initial value to all variables.
     */
    public static DataState getInitialState(double gran, double Tstep, double Treal, double Tdelta) {
        Map<Integer, Double> values = new HashMap<>();

        values.put(x_key, x_init);

        return new DataState(1, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Treal, Tdelta);
    }

}
