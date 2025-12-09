package nl.tue.ABZ2025;

import Scenarios.AIMultipleLanes;
import Scenarios.*;

import java.io.IOException;
import java.util.*;


public class Main {

    // Uncomment the case study you would like to run

    // AI
    public static void main(String[] args) {
        double[] sensorPertubationOffsets = new double[]{0.25};
        double[] invisibilityChances = new double[]{0.25};
        for(int i = 0; i < sensorPertubationOffsets.length; i++){
            new AIMultipleLanes(sensorPertubationOffsets[i], invisibilityChances[i]);
        }
    }

    // ONE LANE TWO VEHICLES
    /*
    public static void main(String[] args) throws IOException{
        try{
            new SingleLaneTwoCars();
        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }
    */

    // ONE LANE THREE VEHICLES
    /*
    public static void main(String[] args) throws IOException{
        try{
            new OneLaneThreeCars();
        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }
    */

    // TWO LANES TWO VEHICLES
    /*
    public static void main(String[] args) throws IOException{
        try{
            new TwoLanesTwoCars();
        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }
    */

}