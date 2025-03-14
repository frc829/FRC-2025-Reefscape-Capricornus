package digilib.elevator;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static digilib.DigiMath.roundToDecimal;

public class ElevatorTelemetry {
    private final DoublePublisher position;
    private final DoublePublisher velocity;
    private final DoublePublisher voltage;
    private final DoublePublisher current;
    private final DoublePublisher setpointPosition;
    private final DoublePublisher setpointVelocity;

    public ElevatorTelemetry(
            String name,
            double minHeightMeters,
            double maxHeightMeters,
            double maxVelocityMPS,
            double maxAccelerationMPSSquared) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getDoubleTopic("Min Height [meters]")
                .publish()
                .set(roundToDecimal(minHeightMeters, 2));
        table.getDoubleTopic("Max Height [meters]")
                .publish()
                .set(roundToDecimal(maxHeightMeters, 2));
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
        this.setpointPosition = table
                .getDoubleTopic("Setpoint Position [meters]")
                .publish();
        this.setpointVelocity = table
                .getDoubleTopic("Setpoint Velocity [mps]")
                .publish();
    }

    public void telemeterize(ElevatorState state) {
        double motorEncoderPositionMeters = state.getMotorEncoderPositionMeters();
        double motorEncoderVelocityMPS = state.getMotorEncoderVelocityMPS();
        double volts = state.getVolts();
        double amps = state.getAmps();
        double setpointPositionMeters = state.getPositionSetpointMeters();
        double setpointVelocityMPS = state.getVelocitySetpointMPS();

        motorEncoderPositionMeters = roundToDecimal(motorEncoderPositionMeters, 2);
        motorEncoderVelocityMPS = roundToDecimal(motorEncoderVelocityMPS, 2);
        volts = roundToDecimal(volts, 2);
        amps = roundToDecimal(amps, 2);
        setpointPositionMeters = roundToDecimal(setpointPositionMeters, 2);
        setpointVelocityMPS = roundToDecimal(setpointVelocityMPS, 2);

        position.set(motorEncoderPositionMeters);
        velocity.set(motorEncoderVelocityMPS);
        voltage.set(volts);
        current.set(amps);
        setpointPosition.set(setpointPositionMeters);
        setpointVelocity.set(setpointVelocityMPS);
    }
}
