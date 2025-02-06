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

package Scenarios;

import it.unicam.quasylab.jspear.*;
import it.unicam.quasylab.jspear.controller.Controller;
import it.unicam.quasylab.jspear.controller.ControllerRegistry;
import it.unicam.quasylab.jspear.controller.ExecController;
import it.unicam.quasylab.jspear.distance.AtomicDistanceExpressionLeq;
import it.unicam.quasylab.jspear.distance.DistanceExpression;
import it.unicam.quasylab.jspear.distance.MaxIntervalDistanceExpression;
import it.unicam.quasylab.jspear.ds.DataState;
import it.unicam.quasylab.jspear.ds.DataStateExpression;
import it.unicam.quasylab.jspear.ds.DataStateUpdate;
import it.unicam.quasylab.jspear.ds.RelationOperator;
import it.unicam.quasylab.jspear.perturbation.AtomicPerturbation;
import it.unicam.quasylab.jspear.perturbation.IterativePerturbation;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.robtl.AtomicRobustnessFormula;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;
import nl.tue.ABZ2025.AI.AiState;
import nl.tue.ABZ2025.AI.Connector;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.*;

public class AIMutipleLanes {

    private static final double RESPONSE_TIME = 1;

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;

    // PERTURBATION PARAMETERS
    private static final int STARTING_STEP = 1;
    private static final int FREQUENCY = 1;
    private static final int TIMES_TO_APPLY = 20;

    private final double sensorPerturbationOffset;
    private final double invisibleCarChance;

    // PARAMETERS FOR ROBUSTNESS FORMULAE WITH MAXIMAL-IN-INTERVAL OPERATORS
    private static final double ETA_CRASH = 0.01; // Maximum acceptable risk of collision
    private static final double ETA_SAFETY_GAP_VIOLATION = 0.01; // Maximum acceptable risk of violating safety gap

    private static final int EVOLUTION_SEQUENCE_SIZE = 25;
    private static final int PERTURBATION_SCALE = 20;
    private static final int STEPS_TO_SAMPLE = 30;
    private static final Connector AI = new Connector("http://127.0.0.1:6000");

    // DATASTATE INDEXES
    private int[] presence;
    private int[] pPresence;
    private int[] xPosition;
    private int[] xSpeed;
    private int[] pxPosition;
    private int[] pxSpeed;
    private int[] yPosition;
    private int[] ySpeed;
    private int[] pyPosition;
    private int[] pySpeed;

    private int observedCarCount;


    public AIMutipleLanes(double sensorPerturbationOffset, double invisibleCarChance){
        this.sensorPerturbationOffset = sensorPerturbationOffset;
        this.invisibleCarChance = invisibleCarChance;
        AiState initialAiState = AI.getInitialState();
        DataState state =  initialAiState.getDataState();
        updateDataStateIndexes(initialAiState);
        ControlledSystem system = new ControlledSystem(getController(), this::getEnvironmentUpdates, state);
        EvolutionSequence sequence = new EvolutionSequence(new SilentMonitor("AIMultipleLanes"), new DefaultRandomGenerator(), rg -> system, EVOLUTION_SEQUENCE_SIZE);

        String experimentFolder ="./ABZ_2025_experiments/AIMultipleLane/";
        String experimentName = "off"+sensorPerturbationOffset+"_cha"+invisibleCarChance+"es"+EVOLUTION_SEQUENCE_SIZE+"ps"+PERTURBATION_SCALE+"s"+STEPS_TO_SAMPLE;

        double[][] atomicDistances = new double[6][STEPS_TO_SAMPLE];
        AtomicDistanceExpressionLeq crashPenalty = new AtomicDistanceExpressionLeq(getCrashPenalty());
        AtomicDistanceExpressionLeq sgViolationPenalty = new AtomicDistanceExpressionLeq(getSafetyGapViolationPenaltyFn());


        // SENSOR PERTURBATION
        Perturbation sensor = getSensorPerturbation(TIMES_TO_APPLY, FREQUENCY);
        EvolutionSequence sensorPerturbedSequence = sequence.apply(sensor, STARTING_STEP, PERTURBATION_SCALE);

        for (int i = 0; i < STEPS_TO_SAMPLE; i++){
            atomicDistances[1][i] = crashPenalty.compute(i, sequence, sensorPerturbedSequence);
            atomicDistances[2][i] = sgViolationPenalty.compute(i, sequence, sensorPerturbedSequence);
        }

        System.out.print("Crash distance for Sensor pert. "+sensorPerturbationOffset+" ");
        System.out.println(Arrays.toString(atomicDistances[1]));
        System.out.print("SG Violation distance for Sensor pert. "+sensorPerturbationOffset+" ");
        System.out.println(Arrays.toString(atomicDistances[2]));

        try (OutputStream fileOutputStream = new FileOutputStream(experimentFolder+"summary_"+experimentName+".txt", true)) {
            printSummary(sensorPerturbedSequence, STEPS_TO_SAMPLE, "SENSOR PERTURBATION Offset: "+ sensorPerturbationOffset, fileOutputStream);
            printSummary(sequence, STEPS_TO_SAMPLE, "UNPERTURBED", fileOutputStream);

            printAtomicDistances(fileOutputStream, "Crash distance for Sensor pert. offset "+sensorPerturbationOffset, atomicDistances[1]);
            printAtomicDistances(fileOutputStream, "SG violation distance for Sensor pert. offset "+sensorPerturbationOffset, atomicDistances[2]);
        } catch (IOException e) {
            e.printStackTrace();
        }


        // INVISIBILITY PERTURBATION
        Perturbation invisibility = getInvisibleCarPerturbation(TIMES_TO_APPLY, FREQUENCY);
        EvolutionSequence invisibilitySequence = sequence.apply(invisibility, STARTING_STEP, PERTURBATION_SCALE);

        for (int i = 0; i < STEPS_TO_SAMPLE; i++){
            atomicDistances[0][i] = crashPenalty.compute(i, sequence, invisibilitySequence);
            atomicDistances[3][i] = sgViolationPenalty.compute(i, sequence, invisibilitySequence);
        }

        System.out.print("Crash distance for invisibility pert. "+invisibleCarChance);
        System.out.println(Arrays.toString(atomicDistances[0]));

        System.out.print("SG Violation distance for invisibility pert. "+invisibleCarChance);
        System.out.println(Arrays.toString(atomicDistances[3]));

        try (OutputStream fileOutputStream = new FileOutputStream(experimentFolder+"summary_"+experimentName+".txt", true)) {
            printSummary(invisibilitySequence, STEPS_TO_SAMPLE, "INVISIBLE CAR Chance:"+ invisibleCarChance, fileOutputStream);

            printAtomicDistances(fileOutputStream, "Crash distance for invisibility pert. offset "+sensorPerturbationOffset, atomicDistances[0]);
            printAtomicDistances(fileOutputStream, "SG violation distance for invisibility pert. "+sensorPerturbationOffset, atomicDistances[3]);
        } catch (IOException e) {
            e.printStackTrace();
        }


        // SPEED PERTURBATION
        Perturbation speedPerturbation = getSpeedPerturbation(TIMES_TO_APPLY, FREQUENCY);
        EvolutionSequence speedPerturbedSequence = sequence.apply(speedPerturbation, STARTING_STEP, PERTURBATION_SCALE);


        //RobustnessFormula PHINoPerturbation = getMaxIntervalCrashFormula(new NonePerturbation());
        //RobustnessFormula PHISensor = getMaxIntervalCrashFormula(sensor);


        for (int i = 0; i < STEPS_TO_SAMPLE; i++){
            atomicDistances[4][i] = crashPenalty.compute(i, sequence, speedPerturbedSequence);
            atomicDistances[5][i] = sgViolationPenalty.compute(i, sequence, speedPerturbedSequence);
        }

        //printSummary(sequence, STEPS_TO_SAMPLE, "UNPERTURBED", System.out);
//        printSummary(speedPerturbedSequence, STEPS_TO_SAMPLE, "SPEED PERTURBATION", System.out);
//        printSummary(sensorPerturbedSequence, STEPS_TO_SAMPLE, "SENSOR PERTURBATION", System.out);
//        printSummary(invisibilitySequence, STEPS_TO_SAMPLE, "INVISIBLE CAR", System.out);
//

        System.out.print("Crash distance for Speed pert. ");
        System.out.println(Arrays.toString(atomicDistances[4]));
        System.out.print("SG Violation distance for Speed pert. ");
        System.out.println(Arrays.toString(atomicDistances[5]));



        try (OutputStream fileOutputStream = new FileOutputStream(experimentFolder+"summary_"+experimentName+".txt", true)) {
            printSummary(speedPerturbedSequence, STEPS_TO_SAMPLE, "SPEED PERTURBATION Offset: "+ sensorPerturbationOffset, fileOutputStream);

            printAtomicDistances(fileOutputStream, "Crash distance for speed pert. offset "+sensorPerturbationOffset, atomicDistances[4]);
            printAtomicDistances(fileOutputStream, "SG violation distance for speed pert. offset "+sensorPerturbationOffset, atomicDistances[5]);
        } catch (IOException e) {
            e.printStackTrace();
        }


//        for(int testStep = 0; testStep < 100; testStep++){
//            System.out.print("Step " + testStep + ":  ");
//            System.out.print("PHISensor "  + new BooleanSemanticsVisitor().eval(PHISensor).eval(100, testStep, sequence));
//            System.out.print(" PHINoPerturbation "  + new BooleanSemanticsVisitor().eval(PHINoPerturbation).eval(100, testStep, sequence));
//            System.out.println();
//        }
    }

    private void updateDataStateIndexes(AiState aiState) {
        presence = aiState.getRealDataStatePresenceIndexes();
        pPresence = aiState.getPerturbedDataStatePresenceIndexes();
        xSpeed = aiState.getRealDataStateXSpeedIndexes();
        xPosition = aiState.getRealDataStateXPositionIndexes();
        pxSpeed = aiState.getPerturbedDataStateXSpeedIndexes();
        pxPosition = aiState.getPerturbedDataStateXPositionIndexes();

        ySpeed = aiState.getRealDataStateYSpeedIndexes();
        yPosition = aiState.getRealDataStateYPositionIndexes();
        pySpeed = aiState.getPerturbedDataStateYSpeedIndexes();
        pyPosition = aiState.getPerturbedDataStateYPositionIndexes();
        observedCarCount = aiState.getCarCount();
    }

    private void printSummary(EvolutionSequence sequence, int stepsToPrint, String title, OutputStream outputStream) {
        int colw = 6;
        PrintWriter writer = new PrintWriter(outputStream);
        try {
            writer.printf("%s%n" + ("%" + colw + "s, ").repeat(16) + "%n", title, "i", "p0", "x0", "v0", "p1", "x1", "v1", "p2", "x2", "v2", "p3", "x3", "v3", "p4", "x4", "v4");

            for (int i = 0; i < stepsToPrint; i++) {
                SampleSet<SystemState> dss = sequence.get(i);
                ArrayList<String> s = new ArrayList<>();
                s.add(String.format("%" + colw + "d,", i));

                for (int j = 0; j < observedCarCount; j++) {
                    int finalJ = j;
                    OptionalDouble p = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(pPresence[finalJ]))).average();
                    s.add(String.format("%" + colw + ".2f,", p.orElse(Double.NaN)));
                    OptionalDouble x = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(pxPosition[finalJ]))).average();
                    s.add(String.format("%" + colw + ".2f,", x.orElse(Double.NaN)));
                    OptionalDouble v = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(pxSpeed[finalJ]))).average();
                    s.add(String.format("%" + colw + ".2f,", v.orElse(Double.NaN)));
                }

                writer.println(String.join(" ", s));
                writer.flush(); // Ensure data is written after every line
            }
        } finally {
            writer.flush(); // Extra flush in case of an out-of-memory situation
        }
    }

    public void printAtomicDistances(OutputStream outputStream, String title, double[] distances) {
        PrintWriter writer = new PrintWriter(outputStream);
        writer.println(title); // Write the provided text

        for (double distance : distances) {
            writer.printf("%.5f ", distance); // Format each double to two decimal places
        }

        writer.println();
        writer.flush();
    }



    private DataStateExpression getCrashPenalty() {
        // Penalizes when the controlled car crashes with the vehicle in front or behind
//        int controlledVehicle = aiState.getControlledVehicleIndex();
//        DataStateExpression penaltyFunction = (ds) -> {
//            for (int other = 0; other < observedCarCount; other++) {
//                double controlledVehiclePosition =  ds.get(position[controlledVehicle]);
//                double otherPosition = ds.get(position[other]);
//                if (other != controlledVehicle && Math.abs(controlledVehiclePosition - otherPosition) <= VEHICLE_LENGTH){
//                   return 1.0;
//                }
//            }
//            return 0.0;
//        };
        // get the corresponding aiState via connector
        return (ds) -> AI.getAiStateFromHistory(ds).getCrashes() > 0 ? 1.0 : 0.0;
    }


    private RobustnessFormula getMaxIntervalCrashFormula(Perturbation perturbation) {
        DistanceExpression distanceExp = new MaxIntervalDistanceExpression(new AtomicDistanceExpressionLeq(getCrashPenalty()), STARTING_STEP, STARTING_STEP + TIMES_TO_APPLY * FREQUENCY);
        return new AtomicRobustnessFormula(perturbation,
                distanceExp,
                RelationOperator.LESS_OR_EQUAL_THAN,
                ETA_CRASH
        );
    }

    private RobustnessFormula getSafetyGapViolationFormula(Perturbation perturbation) {
        DataStateExpression penaltyFunction = getSafetyGapViolationPenaltyFn();
        DistanceExpression distanceExp = new MaxIntervalDistanceExpression(new AtomicDistanceExpressionLeq(penaltyFunction), STARTING_STEP, STARTING_STEP + TIMES_TO_APPLY * FREQUENCY);
        return new AtomicRobustnessFormula(
                perturbation,
                distanceExp,
                RelationOperator.LESS_OR_EQUAL_THAN,
                ETA_SAFETY_GAP_VIOLATION
        );
    }

    private DataStateExpression getSafetyGapViolationPenaltyFn() {
        // Penalizes when the controlled car violates the safety gap with any vehicle in the observation
        return (ds) -> {
            int controlledVehicle = AI.getAiStateFromHistory(ds).getControlledVehicleIndex();
            for (int vehicle = 0; vehicle < observedCarCount; vehicle++) {
                if (vehicle != controlledVehicle) {
                    double vehiclePos = ds.get(xPosition[vehicle]);
                    double safetyGap;
                    if (vehiclePos < 0) { // controlled vehicle is behind
                        safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(xSpeed[controlledVehicle]), ds.get(xSpeed[vehicle]));
                    } else { // controlled vehicle is ahead
                        safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(xSpeed[vehicle]), ds.get(xSpeed[controlledVehicle]));
                    }
                    if (Math.abs(vehiclePos) <= VEHICLE_LENGTH + safetyGap) {
                        return 1.0;
                    }
                }
            }
            return 0.0;
        };
    }

    private static double calculateRSSSafetyDistance(double responseTime, double rearVehicleSpeed, double frontVehicleSpeed){
        /* Formula of safety distance presented by the Responsibility-Sensitive Safety (RSS) model
         * Shalev-Shwartz, S., Shammah, S., Shashua, A.: On a formal model of safe and scalable self-driving cars.
         * CoRR abs/1708.06374 (2017), http://arxiv.org/abs/1708.0637
         */
        double d1 = responseTime*rearVehicleSpeed;
        double d2 = 0.5 * MAX_ACCELERATION*responseTime*responseTime;
        double d3 = Math.pow((rearVehicleSpeed+responseTime*MAX_ACCELERATION),2)/(2*MIN_BRAKE);
        double d4 = - (frontVehicleSpeed*frontVehicleSpeed)/(2*MAX_BRAKE);
        double rssSafetyDistance = Math.max(d1 + d2 + d3 + d4, 0);
        // The RSS model assumes vehicles as points, but ABZ case study vehicles have dimensions.
        // We add the distances from each vehicle's center to its front/rear bumpers.
        return rssSafetyDistance + VEHICLE_LENGTH;
    }

    private Controller getController(){
        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Control", Controller.doTick(registry.reference("Control")));
        return new ExecController(registry.reference("Control"));
    }

    public DataState getEnvironmentUpdates(RandomGenerator rg, DataState ds) {
        AiState newAiState = AI.doStep(ds);
        DataState newState = newAiState.getDataState();
        updateDataStateIndexes(newAiState);
        return newState;
    }


    private Perturbation getSensorPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, this::applySensorPerturbation));
    }

    private DataState applySensorPerturbation(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();

        // Sensor perturbation: add or subtract up to SENSOR_PERTURBATION_OFFSET percent of the original position of non-controlled cars.
        // The perturbation to position x is uniformly sampled from the range [-SENSOR_PERTURBATION_OFFSET * x, SENSOR_PERTURBATION_OFFSET * x]
        int controlledVehicleIndex = AI.getAiStateFromHistory(state).getControlledVehicleIndex();
        for (int i = 0; i < observedCarCount; i++) {
            if(i != controlledVehicleIndex) {
                double perturbedXPosition = (1-(2*rg.nextDouble()-1)* sensorPerturbationOffset)*state.get(xPosition[i]);
                double perturbedYPosition = (1-(2*rg.nextDouble()-1)* sensorPerturbationOffset)*state.get(yPosition[i]);
                //System.out.println("pos["+i+"] original: "+ state.get(position[i]) + " perturbed: " + perturbedXPosition);
                updates.add(new DataStateUpdate(pxPosition[i], perturbedXPosition));
                updates.add(new DataStateUpdate(pyPosition[i], perturbedYPosition));
            }
        }
        return state.apply(updates);
    }

    private Perturbation getSpeedPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, this::applySpeedPerturbation));
    }

    private DataState applySpeedPerturbation(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();

        // Speed perturbation: add or subtract up to SENSOR_PERTURBATION_OFFSET percent of the original speed of non-controlled cars.
        // The perturbation to speed vx is uniformly sampled from the range [-SENSOR_PERTURBATION_OFFSET * x, SENSOR_PERTURBATION_OFFSET * x]
        int controlledVehicleIndex = AI.getAiStateFromHistory(state).getControlledVehicleIndex();
        for (int i = 0; i < observedCarCount; i++) {
            if(i != controlledVehicleIndex) {
                double perturbedSpeed = (1-(2*rg.nextDouble()-1)* sensorPerturbationOffset)*state.get(xSpeed[i]);
                //System.out.println("pos["+i+"] original: "+ state.get(position[i]) + " perturbed: " + perturbedSpeed);
                updates.add(new DataStateUpdate(pxSpeed[i], perturbedSpeed));
            }
        }
        return state.apply(updates);
    }

    private Perturbation getInvisibleCarPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, this::applyInvisibleCarPerturbation));
    }

    private DataState applyInvisibleCarPerturbation(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        // Invisible car perturbation: The "presence" property of non-controlled cars may be set to 0.0 with a INVISIBLE_CAR_CHANCE probability
        int controlledVehicleIndex = AI.getAiStateFromHistory(state).getControlledVehicleIndex();
        for (int i = 0; i < observedCarCount; i++) {
            if(i != controlledVehicleIndex && rg.nextDouble() <= invisibleCarChance) {
                double perturbedPresence = 0.0;
                updates.add(new DataStateUpdate(pPresence[i], perturbedPresence));
            }
        }
        return state.apply(updates);
    }



}