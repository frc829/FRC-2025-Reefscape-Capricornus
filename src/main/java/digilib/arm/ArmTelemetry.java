package digilib.arm;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

public class ArmTelemetry {
    private final DoublePublisher motorEncoderPosition;
    private final DoublePublisher motorEncoderVelocity;
    private final DoublePublisher absoluteEncoderPosition;
    private final DoublePublisher absoluteEncoderVelocity;
    private final DoublePublisher voltage;
    private final DoublePublisher current;
    private final StringPublisher absoluteEncoderStatus;

    public ArmTelemetry(
            String name,
            double minAngleDegrees,
            double maxAngleDegrees,
            double maxVelocityRPS,
            double maxAccelerationRPSSquared) {
        NetworkTable table = getDefault().getTable(name);
        table.getDoubleTopic("Min Angle [deg]")
                .publish()
                .set(minAngleDegrees);
        table.getDoubleTopic("Max Angle [deg]")
                .publish()
                .set(maxAngleDegrees);
        table.getDoubleTopic("Max Velocity [dps]")
                .publish()
                .set(maxVelocityRPS * 360.0);
        table.getDoubleTopic("Max Acceleration [dpss]")
                .publish()
                .set(maxAccelerationRPSSquared * 360);
        motorEncoderPosition = table
                .getDoubleTopic("Motor Encoder Angle [deg]")
                .publish();
        absoluteEncoderPosition = table
                .getDoubleTopic("Absolute Encoder Angle [deg]")
                .publish();
        motorEncoderVelocity = table
                .getDoubleTopic("Motor Encoder Velocity [dps]")
                .publish();
        absoluteEncoderVelocity = table
                .getDoubleTopic("Absolute Encoder Velocity [dps]")
                .publish();
        voltage = table
                .getDoubleTopic("Voltage [volts]")
                .publish();
        current = table
                .getDoubleTopic("Current [amps]")
                .publish();
        absoluteEncoderStatus = table
                .getStringTopic("Absolute Encoder Status")
                .publish();
    }

    public void telemeterize(ArmState state) {
        double motorEncoderPositionDegrees = state.getMotorEncoderPositionDegrees();
        double absoluteEncoderPositionDegrees = state.getAbsoluteEncoderPositionDegrees();
        double motorEncoderVelocityDPS = state.getMotorEncoderVelocityDPS();
        double absoluteEncoderVelocityDPS = state.getAbsoluteEncoderVelocityDPS();
        double volts = state.getVolts();
        double amps = state.getAmps();

        motorEncoderPosition.set(motorEncoderPositionDegrees);
        absoluteEncoderPosition.set(absoluteEncoderPositionDegrees);
        motorEncoderVelocity.set(motorEncoderVelocityDPS);
        absoluteEncoderVelocity.set(absoluteEncoderVelocityDPS);
        voltage.set(volts);
        current.set(amps);
        absoluteEncoderStatus.set(state.getAbsoluteEncoderStatus().name());
    }
}
