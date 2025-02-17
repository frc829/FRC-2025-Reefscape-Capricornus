package digilib;

import jdk.jshell.spi.ExecutionControl;

import java.util.List;

public class DigiMath {

    private DigiMath() {
        // do not instantiate
    }

    public static double roundToDecimal(double value, int places) throws IllegalArgumentException {
        if (places < 0) {
            throw new IllegalArgumentException("Decimal places must be non-negative");
        }

        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }

    public static double standardDeviation(double mean, List<Double> data) throws ExecutionControl.NotImplementedException {
        throw new ExecutionControl.NotImplementedException("not Yet implemented");
    }
}
