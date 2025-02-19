package digilib;

import edu.wpi.first.math.geometry.Rotation2d;

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

    public static double standardDeviation(double mean, List<Double> data) {
        return Math.sqrt(data
                .stream()
                .map(x -> Math.pow(x - mean, 2))
                .reduce(0.0, Double::sum) / data.size());
    }

    public static double standardDeviation(Rotation2d mean, List<Rotation2d> data) {
        return Math.sqrt(data
                .stream()
                .map(rotation2d -> rotation2d.minus(mean))
                .map(Rotation2d::getRadians)
                .map(radians -> Math.pow(radians, 2))
                .reduce(0.0, Double::sum)
                / data.size());
    }
}
