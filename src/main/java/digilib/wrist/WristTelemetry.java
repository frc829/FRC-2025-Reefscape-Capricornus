package digilib.wrist;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static digilib.DigiMath.roundToDecimal;
import static edu.wpi.first.units.Units.*;

public class WristTelemetry {
    // TODO: all fields are private final
    private final NetworkTable wristStateTable;
    private final Angle minAngle;
    private final Angle maxAngle;
    private final AngularVelocity maxVelocity;
    private final AngularAcceleration maxAcceleration;
    private final DoublePublisher timeStamp;
    private final DoublePublisher angle;
    private final DoublePublisher angularVelocity;
    private final DoublePublisher minAnglePublisher;
    private final DoublePublisher maxAnglePublisher;
    private final DoublePublisher maxVelocityPublisher;
    private final DoublePublisher maxAccelerationPublisher;

    private final Mechanism2d wristMechanism;
    private final MechanismLigament2d wristLigamentRight;
    private final MechanismLigament2d wristLigamentLeft;

    public WristTelemetry(
            String name,
            Angle minAngle,
            Angle maxAngle,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.wristStateTable = NetworkTableInstance.getDefault().getTable(name);
        this.timeStamp = wristStateTable.getDoubleTopic("Angle").publish();
        this.angle = wristStateTable.getDoubleTopic("Angle").publish();
        this.angularVelocity = wristStateTable.getDoubleTopic("AngularVelocity").publish();
        this.minAnglePublisher = wristStateTable.getDoubleTopic("MinAngle").publish();
        this.maxAnglePublisher = wristStateTable.getDoubleTopic("MaxAngle").publish();
        this.maxVelocityPublisher = wristStateTable.getDoubleTopic("MaxVelocity").publish();
        this.maxAccelerationPublisher = wristStateTable.getDoubleTopic("MaxAcceleration").publish();

        wristMechanism = new Mechanism2d(1, 1);
        wristLigamentRight = wristMechanism
                .getRoot("WristRoot", 0.5, 0.5)
                .append(new MechanismLigament2d("Wrist Ligament Right", 0.5, 90));
        wristLigamentLeft = wristMechanism
                .getRoot("WristRoot", 0.5, 0.5)
                .append(new MechanismLigament2d("Wrist Ligament Left", 0.5, 270));
        SmartDashboard.putData("Wrist", wristMechanism);
    }

    public void telemeterize(WristState state) {
        minAnglePublisher.set(roundToDecimal(minAngle.in(Degrees), 2));
        maxAnglePublisher.set(roundToDecimal(maxAngle.in(Degrees), 2));
        maxVelocityPublisher.set(roundToDecimal(maxVelocity.in(DegreesPerSecond), 2));
        maxAccelerationPublisher.set(roundToDecimal(maxAcceleration.in(DegreesPerSecondPerSecond), 2));
        angle.set(roundToDecimal(state.getPosition().in(Degrees), 2));
        angularVelocity.set(roundToDecimal(state.getVelocity().in(DegreesPerSecond), 2));
        wristLigamentRight.setAngle(roundToDecimal(90 + state.getPosition().in(Degrees), 2));
        wristLigamentLeft.setAngle(roundToDecimal(270.0 + state.getPosition().in(Degrees), 2));
    }
}
