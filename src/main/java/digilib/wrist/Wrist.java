package digilib.wrist;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class Wrist {

    public record Config(String name,
                                 double reduction,
                                 double startingAngleDegrees,
                                 double minAngleDegrees,
                                 double maxAngleDegrees,
                                 double maxControlVoltage,
                                 double ksVolts,
                                 double kvVoltsPerRPS,
                                 double kaVoltsPerRPSSquared,
                                 double maxVelocityRPS,
                                 double maxAccelerationRPSSquared) {
    }

    private final DoublePublisher motorEncoderPosition;
    private final DoublePublisher motorEncoderVelocity;
    private final DoublePublisher voltage;
    private final DoublePublisher current;

    @SuppressWarnings("resource")
    public Wrist(String name,
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

    public abstract double getMotorEncoderPositionRotations();

    public abstract double getMotorEncoderPositionDegrees();

    public abstract double getMotorEncoderVelocityDPS();

    public abstract void applyPositionRotations(double setpointRotations);

    public abstract void applyVelocity(double setpointScalar);

    public abstract double getVolts();

    public abstract double getAmps();

    public void update() {
        motorEncoderPosition.set(getMotorEncoderPositionDegrees());
        motorEncoderVelocity.set(getMotorEncoderVelocityDPS());
        voltage.set(getVolts());
        current.set(getAmps());
    }

    public abstract void updateSimState(double dt, double supplyVoltage);
}
