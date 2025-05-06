package digilib.motorControllers;

import com.revrobotics.spark.SparkBase;

public class SparkBaseController implements MotorController {

    private final SparkBase sparkBase;

    private SparkBaseController(SparkBase sparkBase) {
        this.sparkBase = sparkBase;
    }

    public static SparkBaseController create(SparkBase sparkBase) {
        return null;
    }

    public static SparkBaseController createForElevator(SparkBase sparkBase) {
        return null;
    }

    public static SparkBaseController createForArm(SparkBase sparkBase) {
        return null;
    }

}
