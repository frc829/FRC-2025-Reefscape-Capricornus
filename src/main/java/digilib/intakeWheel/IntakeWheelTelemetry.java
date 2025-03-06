package digilib.intakeWheel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static digilib.DigiMath.roundToDecimal;

public class IntakeWheelTelemetry {
    private final DoublePublisher motorEncoderVelocity;
    private final DoublePublisher voltage;

    public IntakeWheelTelemetry(
            String name,
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
    }

    public void telemeterize(IntakeWheelState state) {
        double motorEncoderVelocityDPS = state.getMotorEncoderVelocityDPS();
        double volts = state.getVolts();

        motorEncoderVelocityDPS = roundToDecimal(motorEncoderVelocityDPS, 2);
        volts = roundToDecimal(volts, 2);

        motorEncoderVelocity.set(motorEncoderVelocityDPS);
        voltage.set(volts);
    }
}
