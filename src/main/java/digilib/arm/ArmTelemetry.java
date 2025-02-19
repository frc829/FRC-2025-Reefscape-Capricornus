package digilib.arm;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static digilib.DigiMath.roundToDecimal;
import static edu.wpi.first.units.Units.*;

public class ArmTelemetry {
    private final DoublePublisher angle;
    private final DoublePublisher absoluteAngle;
    private final DoublePublisher angularVelocity;
    private final DoublePublisher absoluteVelocity;
    private final DoublePublisher voltage;
    private final DoublePublisher timestamp;
    private final StringPublisher absoluteEncoderStatus;
    private final MechanismLigament2d ligament;

    public ArmTelemetry(
            String name,
            Angle minAngle,
            Angle maxAngle,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getDoubleTopic("Min Angle").publish().set(roundToDecimal(minAngle.in(Degrees), 2));
        table.getDoubleTopic("Max Angle").publish().set(roundToDecimal(maxAngle.in(Degrees), 2));
        table.getDoubleTopic("Max Velocity").publish().set(roundToDecimal(maxVelocity.in(DegreesPerSecond), 2));
        table.getDoubleTopic("Max Acceleration").publish().set(roundToDecimal(maxAcceleration.in(DegreesPerSecondPerSecond), 2));
        angle = table.getDoubleTopic("Angle").publish();
        absoluteAngle = table.getDoubleTopic("Absolute Angle").publish();
        angularVelocity = table.getDoubleTopic("Velocity").publish();
        absoluteVelocity = table.getDoubleTopic("AbsoluteVelocity").publish();
        voltage = table.getDoubleTopic("Voltage").publish();
        timestamp = table.getDoubleTopic("Timestamp").publish();
        absoluteEncoderStatus = table.getStringTopic("Absolute Encoder Status").publish();

        Mechanism2d armMechanism = new Mechanism2d(1, 1);
        ligament = armMechanism
                .getRoot("ArmRoot", 0.5, 0.5)
                .append(new MechanismLigament2d("Arm", 0.5, 0));
        SmartDashboard.putData("Arm", armMechanism);
    }

    public void telemeterize(ArmState state) {
        angle.set(roundToDecimal(state.getAngle().in(Degrees), 2));
        absoluteAngle.set(roundToDecimal(state.getAbsolutePosition().in(Degrees), 2));
        angularVelocity.set(roundToDecimal(state.getVelocity().in(DegreesPerSecond), 2));
        absoluteVelocity.set(roundToDecimal(state.getAbsoluteVelocity().in(DegreesPerSecond), 2));
        voltage.set(roundToDecimal(state.getVoltage().baseUnitMagnitude(), 2));
        timestamp.set(roundToDecimal(state.getTimestamp().baseUnitMagnitude(), 2));
        absoluteEncoderStatus.set(state.getAbsoluteEncoderStatus());
        ligament.setAngle(roundToDecimal(state.getAngle().in(Degrees), 2));
    }
}
