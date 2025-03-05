package digilib.arm;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;

import static digilib.DigiMath.roundToDecimal;
import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

public class ArmTelemetry {
    private final DoublePublisher motorEncoderPosition;
    private final DoublePublisher motorEncoderVelocity;
    private final DoublePublisher absoluteEncoderPosition;
    private final DoublePublisher absoluteEncoderVelocity;
    private final DoublePublisher voltage;
    private final StringPublisher absoluteEncoderStatus;

    private double motorEncoderPositionDegrees;
    private double motorEncoderVelocityDPS;
    private double absoluteEncoderPositionDegrees;
    private double absoluteEncoderVelocityDPS;
    private double volts;

    public ArmTelemetry(
            String name,
            double minAngleRotations,
            double maxAngleRotations,
            double maxVelocityRPS,
            double maxAccelerationRPSSquared) {
        NetworkTable table = getDefault().getTable(name);
        table.getDoubleTopic("Min Angle [deg]")
                .publish()
                .set(roundToDecimal(minAngleRotations * 360, 2));
        table.getDoubleTopic("Max Angle [deg]")
                .publish()
                .set(roundToDecimal(maxAngleRotations * 360, 2));
        table.getDoubleTopic("Max Velocity [dps]")
                .publish()
                .set(roundToDecimal(maxVelocityRPS * 360, 2));
        table.getDoubleTopic("Max Acceleration [dpss]")
                .publish()
                .set(roundToDecimal(maxAccelerationRPSSquared * 360, 2));
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
        absoluteEncoderStatus = table
                .getStringTopic("Absolute Encoder Status")
                .publish();
    }

    public void telemeterize(ArmState state) {
        motorEncoderPositionDegrees = state.getMotorEncoderPositionDegrees();
        absoluteEncoderPositionDegrees = state.getAbsoluteEncoderPositionDegrees();
        motorEncoderVelocityDPS = state.getMotorEncoderVelocityDPS() * 360;
        absoluteEncoderVelocityDPS = state.getAbsoluteEncoderVelocityDPS();
        volts = state.getVoltage();

        motorEncoderPositionDegrees = roundToDecimal(motorEncoderPositionDegrees, 2);
        absoluteEncoderPositionDegrees = roundToDecimal(absoluteEncoderPositionDegrees, 2);
        motorEncoderVelocityDPS = roundToDecimal(motorEncoderVelocityDPS, 2);
        absoluteEncoderVelocityDPS = roundToDecimal(absoluteEncoderVelocityDPS, 2);
        volts = roundToDecimal(volts, 2);

        motorEncoderPosition.set(motorEncoderPositionDegrees);
        absoluteEncoderPosition.set(motorEncoderPositionDegrees);
        motorEncoderVelocity.set(motorEncoderVelocityDPS);
        absoluteEncoderVelocity.set(motorEncoderVelocityDPS);
        voltage.set(volts);
        absoluteEncoderStatus.set(state.getAbsoluteEncoderStatus().name());
    }
}
