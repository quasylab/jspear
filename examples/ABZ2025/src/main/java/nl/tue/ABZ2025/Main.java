package nl.tue.ABZ2025;

import Scenarios.SingleLaneMultipleCars;
import Scenarios.SingleLaneTwoCars;
import Scenarios.TwoLanesTwoCars;

import java.io.IOException;

public class Main {
    public static void main(String[] args) throws IOException {
        try{
            new SingleLaneTwoCars();
        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }
}