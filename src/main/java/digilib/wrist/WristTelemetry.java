package digilib.wrist;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getDoubleTopic("Min Angle [deg]")
                .publish()
                .set(minAngleDegrees);
        table.getDoubleTopic("Max Angle [deg]")
                .publish()
                .set(maxAngleDegrees);
        table.getDoubleTopic("Max Velocity [dps]")
                .publish()
                .set(maxVelocityRPS * 360);
        table.getDoubleTopic("Max Acceleration [dpss]")
                .publish()
                .set(maxAccelerationRPSSquared * 360);
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

        motorEncoderPosition.set(motorEncoderPositionDegrees);
        motorEncoderVelocity.set(motorEncoderVelocityDPS);
        voltage.set(volts);
        current.set(amps);
    }
}
