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
import it.unicam.quasylab.jspear.perturbation.NonePerturbation;
import it.unicam.quasylab.jspear.perturbation.Perturbation;
import it.unicam.quasylab.jspear.robtl.AtomicRobustnessFormula;
import it.unicam.quasylab.jspear.robtl.BooleanSemanticsVisitor;
import it.unicam.quasylab.jspear.robtl.RobustnessFormula;
import nl.tue.ABZ2025.AI.AiState;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.*;

import nl.tue.ABZ2025.AI.Connector;

public class AISingleLaneMultipleCars {

    private static final double RESPONSE_TIME = 1;

    private  static final int NUMBER_OF_VEHICLES = 3;

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double MAX_ACCEL_OFFSET = 5;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;
    private static final double IDLE_DELTA = 1;


    // PERTURBATION PARAMETERS
    private static final int STARTING_STEP = 4;
    private static final int FREQUENCY = 1;
    private static final int TIMES_TO_APPLY = 10;

    private static final double SENSOR_PERTURBATION_OFFSET = 0.05;
    private static final double BRAKE_CHECK_CHANCE = 0.4;

    // ROBUSTNESS FORMULAE PARAMETERS
    private static final double ETA_CRASH = 0.01; // Maximum acceptable risk of collision
    private static final double ETA_SAFETY_GAP_VIOLATION = 0.2; // Maximum acceptable risk of violating safety gap

    private static final int EVOLUTION_SEQUENCE_SIZE = 1;
    private static final int STEPS_TO_SAMPLE = 30;
    private static final Connector AI = new Connector("http://127.0.0.1:5000");

    private AiState aiState;
    private int[] position;
    private int[] speed;
    private int[] presence;
    private int[] pPosition;
    private int[] pSpeed;
    private int[] pPresence;
    private int observedCarCount;

    public AISingleLaneMultipleCars(){
        aiState = AI.getInitialState();
        updateAIState(aiState);
        DataState state =  aiState.getDataState();
        ControlledSystem system = new ControlledSystem(getController(), this::getEnvironmentUpdates, state);
        EvolutionSequence sequence = new EvolutionSequence(new SilentMonitor("Vehicle"), new DefaultRandomGenerator(), rg -> system, EVOLUTION_SEQUENCE_SIZE);
        printSummary(sequence, STEPS_TO_SAMPLE, "UNPERTURBED");
        Perturbation sensor = getSensorPerturbation(TIMES_TO_APPLY, FREQUENCY);
//        Perturbation brakeChecker = getBreakCheckPerturbation(TIMES_TO_APPLY, FREQUENCY);
//
        EvolutionSequence sensorPerturbedSequence = sequence.apply(sensor, STARTING_STEP, EVOLUTION_SEQUENCE_SIZE);
//        EvolutionSequence brakeCheckerSequence = sequence.apply(brakeChecker, STARTING_STEP, EVOLUTION_SEQUENCE_SIZE);
//
//
        printSummary(sensorPerturbedSequence, STEPS_TO_SAMPLE, "SENSOR PERTURBATION");
//        printSummary(brakeCheckerSequence, STEPS_TO_SAMPLE, "BRAKE CHECKER");
//
        RobustnessFormula PHINoPerturbation = getCrashFormula(new NonePerturbation());
       RobustnessFormula PHISensor = getCrashFormula(sensor);
//        RobustnessFormula PHIDrunkDriver = getSafetyGapViolationFormula(sensorPerturbated
//       );
//
        PHINoPerturbation = getCrashFormula(new NonePerturbation());
        PHISensor = getCrashFormula(sensor);
//        PHIDrunkDriver = getCrashFormula(sensorPerturbated
//       );
//
        for(int testStep = 0; testStep < 100; testStep++){
            System.out.print("Step " + testStep + ":  ");
            System.out.print("PHISensor "  + new BooleanSemanticsVisitor().eval(PHISensor).eval(100, testStep, sequence));
//            System.out.print(" PHIDrunkDriver "  + new BooleanSemanticsVisitor().eval(PHIDrunkDriver).eval(100, testStep, sequence));
            System.out.print(" PHINoPerturbation "  + new BooleanSemanticsVisitor().eval(PHINoPerturbation).eval(100, testStep, sequence));
            System.out.println();
        }
    }

    private void updateAIState(AiState aiState) {
        this.aiState = aiState;
        speed = aiState.getRealDataStateSpeedIndexes();
        position = aiState.getRealDataStatePositionIndexes();
        presence = aiState.getRealDataStatePresenceIndexes();
        pSpeed = aiState.getPerturbedDataStateSpeedIndexes();
        pPosition = aiState.getPerturbedDataStatePositionIndexes();
        pPresence = aiState.getPerturbedDataStatePresenceIndexes();
        observedCarCount = aiState.getCarCount();
    }

    private void printSummary(EvolutionSequence sequence, int stepsToPrint, String title){
        System.out.printf("%s%n%5s, %5s, %5s, %5s, %5s, %5s, %5s, %5s, %5s, %5s, %5s%n", title, "i", "x0", "v0", "x1", "v1", "x2", "v2","x3", "v3","x4", "v4");

        for (int i = 0; i < stepsToPrint; i++) {
            SampleSet<SystemState> dss = sequence.get(i);
            ArrayList<String> s = new ArrayList<>();
            s.add(String.format("%5d,",i));
            for (int j = 0; j < observedCarCount; j++) {
                int finalJ = j;
                OptionalDouble x = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(position[finalJ]))).average();
                s.add(String.format("%5.2f,", x.getAsDouble()));
                OptionalDouble v = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(speed[finalJ]))).average();
                s.add(String.format("%5.2f,", v.getAsDouble()));
            }
            System.out.println(String.join(" ", s));
        }
    }



    private RobustnessFormula getCrashFormula(Perturbation perturbation) {
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

        DataStateExpression penaltyFunction = (ds) -> aiState.getCrashes() > 0 ? 1.0 : 0.0;

        DistanceExpression distanceExp = new MaxIntervalDistanceExpression(new AtomicDistanceExpressionLeq(penaltyFunction), STARTING_STEP, STARTING_STEP + TIMES_TO_APPLY * FREQUENCY);
        return new AtomicRobustnessFormula(perturbation,
                distanceExp,
                RelationOperator.LESS_OR_EQUAL_THAN,
                ETA_CRASH
        );
    }

    private RobustnessFormula getSafetyGapViolationFormula(Perturbation perturbation) {
        // Penalizes when the controlled car violates de safety gap with any vehicle in the observation
        int controlledVehicle = aiState.getControlledVehicleIndex();
        DataStateExpression penaltyFunction = (ds) -> {
            for (int other = 0; other < observedCarCount; other++) {
                double controlledVehiclePosition =  ds.get(position[controlledVehicle]);
                double otherPosition = ds.get(position[controlledVehicle - 1]);
                double safetyGap;
                if (otherPosition > controlledVehiclePosition){ // controlled vehicle is behind
                    safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(speed[controlledVehicle]), ds.get(speed[other]));
                } else {
                    safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(speed[other]), ds.get(speed[controlledVehicle]));
                }
                if (other != controlledVehicle && Math.abs(controlledVehiclePosition - otherPosition) <= VEHICLE_LENGTH + safetyGap) {
                    return 1.0;
                }
            }
            return 0.0;
        };
        DistanceExpression distanceExp = new MaxIntervalDistanceExpression(new AtomicDistanceExpressionLeq(penaltyFunction), STARTING_STEP, STARTING_STEP + TIMES_TO_APPLY * FREQUENCY);
        return new AtomicRobustnessFormula(
                perturbation,
                distanceExp,
                RelationOperator.LESS_OR_EQUAL_THAN,
                ETA_SAFETY_GAP_VIOLATION
        );
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

    public DataState getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        AiState newAiState = AI.doStep(aiState, state);
        observedCarCount = aiState.getCarCount();
        updateAIState(newAiState);
        return newAiState.getDataState();
    }


    private Perturbation getSensorPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, this::applySensorPerturbation));
    }

    private DataState applySensorPerturbation(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();

        // Sensor perturbation: add or subtract up to SENSOR_PERTURBATION_OFFSET percent of the original position of non-controlled cars.
        // The perturbation to position x is uniformly sampled from the range [-SENSOR_PERTURBATION_OFFSET * x, SENSOR_PERTURBATION_OFFSET * x]
        for (int i = 0; i < observedCarCount; i++) {
            if(i != aiState.getControlledVehicleIndex()) {
                double perturbedPosition = (1-(2*rg.nextDouble()-1)*SENSOR_PERTURBATION_OFFSET)*state.get(position[i]);
                // System.out.println("pos["+i+"] original: "+ state.get(position[i]) + " perturbed: " + perturbedPosition);
                updates.add(new DataStateUpdate(pPosition[i], perturbedPosition));
            }
        }
        return state.apply(updates);
    }



}