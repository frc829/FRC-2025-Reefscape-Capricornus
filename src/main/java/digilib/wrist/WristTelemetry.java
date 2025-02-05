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
                .append(new MechanismLigament2d("Wrist Ligament Right", 0.5, 0));
        wristLigamentLeft = wristMechanism
                .getRoot("WristRoot", 0.5, 0.5)
                .append(new MechanismLigament2d("Wrist Ligament Left", 0.5, 180));
        SmartDashboard.putData("Wrist", wristMechanism);
    }

    public void telemeterize(WristState state) {
        minAnglePublisher.set(minAngle.in(Degrees));
        maxAnglePublisher.set(maxAngle.in(Degrees));
        maxVelocityPublisher.set(maxVelocity.in(DegreesPerSecond));
        maxAccelerationPublisher.set(maxAcceleration.in(DegreesPerSecondPerSecond));






//        timeStamp.set(state.getTimeStamp().in(Seconds));






        angle.set(state.getPosition().in(Degrees));
        angularVelocity.set(state.getVelocity().in(DegreesPerSecond));
        wristLigamentRight.setAngle(state.getPosition().in(Degrees));
        wristLigamentLeft.setAngle(180.0 + state.getPosition().in(Degrees));
    }
}
