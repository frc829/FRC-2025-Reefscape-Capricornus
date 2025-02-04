package frc.robot.mechanisms.arm;

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
        minAnglePublisher.set(minAngle.in(Degrees));
        maxAnglePublisher.set(maxAngle.in(Degrees));
        maxVelocityPublisher.set(maxVelocity.in(DegreesPerSecond));
        maxAccelerationPublisher.set(maxAcceleration.in(DegreesPerSecondPerSecond));
        timestamp.set(state.getTimestamp().in(Seconds));
        angle.set(state.getPosition().in(Degrees));
        angularVelocity.set(state.getVelocity().in(DegreesPerSecond));


        armLigament.setAngle(state.getPosition().in(Degrees));
    }
}
