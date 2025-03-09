package digilib.wrist;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;

import static digilib.DigiMath.roundToDecimal;
import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

public class WristTelemetry {
    private final DoublePublisher motorEncoderPosition;
    private final DoublePublisher motorEncoderVelocity;
    private final DoublePublisher voltage;
    private final DoublePublisher current;

    public WristTelemetry(
            String name,
            double minAngleDegrees,
            double maxAngleDegrees,
            double maxVelocityRPS,
            double maxAccelerationRPSSquared) {
        NetworkTable table = getDefault().getTable(name);
        table.getDoubleTopic("Min Angle [deg]")
                .publish()
                .set(roundToDecimal(minAngleDegrees, 2));
        table.getDoubleTopic("Max Angle [deg]")
                .publish()
                .set(roundToDecimal(maxAngleDegrees, 2));
        table.getDoubleTopic("Max Velocity [dps]")
                .publish()
                .set(roundToDecimal(maxVelocityRPS * 360, 2));
        table.getDoubleTopic("Max Acceleration [dpss]")
                .publish()
                .set(roundToDecimal(maxAccelerationRPSSquared * 360, 2));
        motorEncoderPosition = table
                .getDoubleTopic("Motor Encoder Angle [deg]")
                .publish();
        motorEncoderVelocity = table
                .getDoubleTopic("Motor Encoder Velocity [dps]")
                .publish();
        voltage = table
                .getDoubleTopic("Voltage [volts]")
                .publish();
        current = table
                .getDoubleTopic("Current [amps]")
                .publish();
    }

    public void telemeterize(WristState state) {
        double motorEncoderPositionDegrees = state.getMotorEncoderPositionDegrees();
        double motorEncoderVelocityDPS = state.getMotorEncoderVelocityDPS();
        double volts = state.getVolts();
        double amps = state.getAmps();

        motorEncoderPositionDegrees = roundToDecimal(motorEncoderPositionDegrees, 2);
        motorEncoderVelocityDPS = roundToDecimal(motorEncoderVelocityDPS, 2);
        volts = roundToDecimal(volts, 2);
        amps = roundToDecimal(amps, 2);

        motorEncoderPosition.set(motorEncoderPositionDegrees);
        motorEncoderVelocity.set(motorEncoderVelocityDPS);
        voltage.set(volts);
        current.set(amps);
    }
}
