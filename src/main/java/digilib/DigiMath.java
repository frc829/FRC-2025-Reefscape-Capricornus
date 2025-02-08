package digilib;

public class DigiMath {

    private DigiMath() {
        // do not instantiate
    }

    public static double roundToDecimal(double value, int places) throws IllegalArgumentException {
        if (places < 0){
            throw new IllegalArgumentException("Decimal places must be non-negative");
        }

        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }
}
