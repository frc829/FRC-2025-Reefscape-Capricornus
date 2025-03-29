package digilib.intakeWheel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class IntakeWheel {

    public record Config(String name,
                                       double reduction,
                                       double maxControlVoltage,
                                       double ksVolts,
                                       double kvVoltsPerRPS,
                                       double kaVoltsPerRPSSquared,
                                       double maxVelocityRPS,
                                       double maxAccelerationRPSSquared) {
    }

    private final DoublePublisher motorEncoderVelocity;
    private final DoublePublisher voltage;
    private final DoublePublisher current;

    @SuppressWarnings("resource")
    IntakeWheel(String name,
                double maxVelocityRPS,
                double maxAccelerationRPSSquared) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getDoubleTopic("Max Velocity [dps]")
                .publish()
                .set(maxVelocityRPS * 360);
        table.getDoubleTopic("Max Acceleration [dpss]")
                .publish()
                .set(maxAccelerationRPSSquared * 360);
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

    public abstract double getMotorEncoderVelocityDPS();

    public abstract void applyMotorEncoderVelocity(double setpointScalar);

    public abstract double getVolts();

    public abstract void applyVolts(double volts);

    public abstract double getAmps();

    public void update() {
        motorEncoderVelocity.set(getMotorEncoderVelocityDPS());
        voltage.set(getVolts());
        current.set(getAmps());
    }

    public abstract void updateSimState(double dtSeconds, double supplyVoltage);
}
