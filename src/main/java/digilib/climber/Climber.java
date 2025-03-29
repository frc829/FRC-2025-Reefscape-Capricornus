package digilib.climber;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class Climber {

    public record Config(String name,
                                   double reduction,
                                   double drumRadiusMeters,
                                   double startingLengthMeters,
                                   double minLengthMeters,
                                   double maxLengthMeters,
                                   double maxControlVoltage,
                                   double ksVolts,
                                   double kvVoltsPerMPS,
                                   double kaVoltsPerMPSSquared,
                                   double maxVelocityMPS,
                                   double maxAccelerationMPSSquared) {
    }

    private final DoublePublisher position;
    private final DoublePublisher velocity;
    private final DoublePublisher voltage;
    private final DoublePublisher current;

    @SuppressWarnings("resource")
    public Climber(String name,
                   double minLengthMeters,
                   double maxLengthMeters,
                   double maxVelocityMPS,
                   double maxAccelerationMPSSquared) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getDoubleTopic("Min Length [meters]")
                .publish()
                .set(minLengthMeters);
        table.getDoubleTopic("Max Length [meters]")
                .publish()
                .set(maxLengthMeters);
        table.getDoubleTopic("Max Velocity [mps]")
                .publish()
                .set(maxVelocityMPS);
        table.getDoubleTopic("Max Acceleration[mpss] ")
                .publish()
                .set(maxAccelerationMPSSquared);
        this.position = table
                .getDoubleTopic("Height [meters]")
                .publish();
        this.velocity = table
                .getDoubleTopic("Velocity [mps]")
                .publish();
        this.voltage = table
                .getDoubleTopic("Voltage [volts]")
                .publish();
        this.current = table
                .getDoubleTopic("Current [amps]")
                .publish();
    }

    public abstract double getMotorEncoderPositionMeters();

    public abstract double getMotorEncoderVelocityMetersPerSecond();

    public abstract double getVolts();

    public abstract void applyVoltage(double volts);

    public abstract double getAmps();

    public void update() {
        position.set(getMotorEncoderPositionMeters());
        velocity.set(getMotorEncoderVelocityMetersPerSecond());
        voltage.set(getVolts());
        current.set(getAmps());
    }

    public abstract void updateSimState(double dt, double supplyVoltage);
}
