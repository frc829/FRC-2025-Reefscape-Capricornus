package digilib.climber;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static digilib.DigiMath.roundToDecimal;

public class ClimberTelemetry {
    private final DoublePublisher position;
    private final DoublePublisher velocity;
    private final DoublePublisher voltage;
    private final DoublePublisher current;

    public ClimberTelemetry(
            String name,
            double minLengthMeters,
            double maxLengthMeters,
            double maxVelocityMPS,
            double maxAccelerationMPSSquared) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getDoubleTopic("Min Length [meters]")
                .publish()
                .set(roundToDecimal(minLengthMeters, 2));
        table.getDoubleTopic("Max Length [meters]")
                .publish()
                .set(roundToDecimal(maxLengthMeters, 2));
        table.getDoubleTopic("Max Velocity [mps]")
                .publish()
                .set(roundToDecimal(maxVelocityMPS, 2));
        table.getDoubleTopic("Max Acceleration[mpss] ")
                .publish()
                .set(roundToDecimal(maxAccelerationMPSSquared, 2));
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

    public void telemeterize(ClimberState state) {
        double motorEncoderPositionMeters = state.getMotorEncoderPositionMeters();
        double motorEncoderVelocityMPS = state.getMotorEncoderVelocityMetersPerSecond();
        double volts = state.getVolts();
        double amps = state.getAmps();

        motorEncoderPositionMeters = roundToDecimal(motorEncoderPositionMeters, 2);
        motorEncoderVelocityMPS = roundToDecimal(motorEncoderVelocityMPS, 2);
        volts = roundToDecimal(volts, 2);
        amps = roundToDecimal(amps, 2);

        position.set(motorEncoderPositionMeters);
        velocity.set(motorEncoderVelocityMPS);
        voltage.set(volts);
        current.set(amps);
    }
}
