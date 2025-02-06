package nl.tue.ABZ2025;

import Scenarios.AISingleLane;
import Scenarios.AIMutipleLanes;

public class Main {
    public static void main(String[] args) {
//        double[] sensorPertubationOffsets = new double[]{0.001, 0.01, 0.05, 0.1, 0.25, 0.5, 0.8};
//        double[] invisibilityChances = new double[]{0.001, 0.01, 0.05, 0.1, 0.25, 0.5};
//        double[] sensorPertubationOffsets = new double[]{0.25, 0.5, 0.8};
//        double[] invisibilityChances = new double[]{0.25, 0.5};
        double[] sensorPertubationOffsets = new double[]{0.25};
        double[] invisibilityChances = new double[]{0.25};
        for(int i = 0; i < sensorPertubationOffsets.length; i++){
            new AIMutipleLanes(sensorPertubationOffsets[i], invisibilityChances[i]);
        }
    }
}