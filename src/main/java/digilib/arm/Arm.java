package digilib.arm;

import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;

import static edu.wpi.first.networktables.NetworkTableInstance.getDefault;

public abstract class Arm {

    public enum AbsoluteEncoderStatus {
        GOOD,
        OK,
        BAD,
        REALLY_BAD;

        public static AbsoluteEncoderStatus fromMagnetHealthValue(MagnetHealthValue magnetHealthValue) {
            return switch (magnetHealthValue) {
                case Magnet_Green -> GOOD;
                case Magnet_Orange -> OK;
                case Magnet_Red -> BAD;
                default -> REALLY_BAD;
            };
        }
    }

    public record Config(String name,
                               double reduction,
                               double startingAngleDegrees,
                               double minAngleDegrees,
                               double maxAngleDegrees,
                               double ksVolts,
                               double kgVolts,
                               double kvVoltsPerRPS,
                               double kaVoltsPerRPSSquared,
                               double maxVelocityRPS,
                               double maxAccelerationRPSSquared) {
    }

    private final DoublePublisher motorEncoderPosition;
    private final DoublePublisher motorEncoderVelocity;
    private final DoublePublisher absoluteEncoderPosition;
    private final DoublePublisher absoluteEncoderVelocity;
    private final DoublePublisher voltage;
    private final DoublePublisher current;
    private final StringPublisher absoluteEncoderStatus;

    public Arm(
            String name,
            double minAngleDegrees,
            double maxAngleDegrees,
            double maxVelocityRPS,
            double maxAccelerationRPSSquared){
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

    public abstract double getMotorEncoderPositionDegrees();

    public abstract double getMotorEncoderVelocityDPS();

    public abstract double getAbsoluteEncoderPositionRotations();

    public abstract double getAbsoluteEncoderPositionDegrees();

    public abstract double getAbsoluteEncoderVelocityDPS();

    public abstract double getVolts();

    public abstract double getAmps();

    public abstract String getAbsoluteEncoderStatus();

    public abstract void applyPosition(double setpointRotations);

    public abstract void applyVelocity(double setpointScalar);

    public void update(){
        motorEncoderPosition.set(getMotorEncoderPositionDegrees());
        motorEncoderVelocity.set(getMotorEncoderVelocityDPS());
        absoluteEncoderPosition.set(getAbsoluteEncoderPositionDegrees());
        absoluteEncoderVelocity.set(getAbsoluteEncoderVelocityDPS());
        voltage.set(getVolts());
        current.set(getAmps());
        absoluteEncoderStatus.set(getAbsoluteEncoderStatus());
    }

    public abstract void updateSimState(double dt, double supplyVoltage);
}
