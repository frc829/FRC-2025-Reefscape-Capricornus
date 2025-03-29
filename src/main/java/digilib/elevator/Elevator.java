package digilib.elevator;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class Elevator {

    public record Config(String name,
                                    double reduction,
                                    double drumRadiusMeters,
                                    double startingHeightMeters,
                                    double minHeightMeters,
                                    double maxHeightMeters,
                                    double maxControlVoltage,
                                    double ksVolts,
                                    double kgVolts,
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
    public Elevator(
            String name,
            double minHeightMeters,
            double maxHeightMeters,
            double maxVelocityMPS,
            double maxAccelerationMPSSquared){
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getDoubleTopic("Min Height [meters]")
                .publish()
                .set(minHeightMeters);
        table.getDoubleTopic("Max Height [meters]")
                .publish()
                .set(maxHeightMeters);
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

    public abstract double getMotorEncoderVelocityMPS();

    public abstract double getVolts();

    public abstract double getAmps();

    public abstract void applyPosition(double setpointMeters);

    public abstract void applyVelocity(double setpointScalar);

    public abstract void applyVoltage(double volts);

    public void update(){
        position.set(getMotorEncoderPositionMeters());
        velocity.set(getMotorEncoderVelocityMPS());
        voltage.set(getVolts());
        current.set(getAmps());
    }

    public abstract void updateSimState(double dt, double supplyVoltage);
}
