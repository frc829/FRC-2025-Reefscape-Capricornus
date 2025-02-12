package digilib.arm;

import digilib.DigiMath;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static digilib.DigiMath.*;
import static edu.wpi.first.units.Units.*;

public class ArmTelemetry {
    private final NetworkTable armStateTable;
    private final Angle minAngle;
    private final Angle maxAngle;
    private final AngularVelocity maxVelocity;
    private final AngularAcceleration maxAcceleration;
    private final DoublePublisher timestamp;
    private final DoublePublisher angle;
    private final DoublePublisher angularVelocity;
    private final DoublePublisher minAnglePublisher;
    private final DoublePublisher maxAnglePublisher;
    private final DoublePublisher maxVelocityPublisher;
    private final DoublePublisher maxAccelerationPublisher;
    private final DoublePublisher absoluteAnglePublisher;
    private final DoublePublisher absoluteVelocityPublisher;
    private final Mechanism2d armMechanism;
    private final MechanismLigament2d armLigament;

    public ArmTelemetry(
            String name,
            Angle minAngle,
            Angle maxAngle,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        this.armStateTable = NetworkTableInstance.getDefault().getTable(name);
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.timestamp = armStateTable.getDoubleTopic("Timestamp").publish();
        this.angle = armStateTable.getDoubleTopic("Angle").publish();
        this.angularVelocity = armStateTable.getDoubleTopic("AngularVelocity").publish();
        this.absoluteAnglePublisher = armStateTable.getDoubleTopic("AbsoluteAngle").publish();
        this.absoluteVelocityPublisher = armStateTable.getDoubleTopic("AbsoluteVelocity").publish();
        this.minAnglePublisher = armStateTable.getDoubleTopic("MinAngle").publish();
        this.maxAnglePublisher = armStateTable.getDoubleTopic("MaxAngle").publish();
        this.maxVelocityPublisher = armStateTable.getDoubleTopic("MaxVelocity").publish();
        this.maxAccelerationPublisher = armStateTable.getDoubleTopic("MaxAcceleration").publish();
        armMechanism = new Mechanism2d(1, 1);
        armLigament = armMechanism
                .getRoot("ArmRoot", 0.5, 0.5)
                .append(new MechanismLigament2d("Arm", 0.5, 0));
        SmartDashboard.putData("Arm", armMechanism);
    }

    public void telemeterize(ArmState state) {
        minAnglePublisher.set(roundToDecimal(minAngle.in(Degrees), 2));
        maxAnglePublisher.set(roundToDecimal(maxAngle.in(Degrees), 2));
        this.absoluteAnglePublisher.set(roundToDecimal(state.getAbsolutePosition().in(Degrees), 2));
        this.absoluteVelocityPublisher.set(roundToDecimal(state.getAbsoluteVelocity().in(DegreesPerSecond), 2));
        maxVelocityPublisher.set(roundToDecimal(maxVelocity.in(DegreesPerSecond), 2));
        maxAccelerationPublisher.set(roundToDecimal(maxAcceleration.in(DegreesPerSecondPerSecond), 2));
        timestamp.set(roundToDecimal(state.getTimestamp().in(Seconds), 2));
        angle.set(roundToDecimal(state.getPosition().in(Degrees), 2));
        angularVelocity.set(roundToDecimal(state.getVelocity().in(DegreesPerSecond), 2));
        armLigament.setAngle(roundToDecimal(state.getPosition().in(Degrees), 2));
    }
}
