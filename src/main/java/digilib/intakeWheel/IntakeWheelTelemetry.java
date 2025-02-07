package digilib.intakeWheel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class IntakeWheelTelemetry {
    private final NetworkTable intakeStateTable;
    private final LinearVelocity maxVelocity;
    private final LinearAcceleration maxAcceleration;
    private final DoublePublisher timestamp;
    private final DoublePublisher velocity;
    private final DoublePublisher maxVelocityPublisher;
    private final DoublePublisher maxAccelerationPublisher;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d ligament;
    private final MutAngle ligamentAngle = Degrees.mutable(0.0);
    private final Distance wheelRadius;
    private double lastTime = 0.0;
    private double time = 0.0;


    public IntakeWheelTelemetry(
            String name,
            Distance wheelRadius,
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration) {
        this.wheelRadius = wheelRadius;
        this.maxVelocity = maxVelocity;
        this.intakeStateTable = NetworkTableInstance.getDefault().getTable(name);
        this.timestamp = intakeStateTable.getDoubleTopic("Timestamp").publish();
        this.velocity = intakeStateTable.getDoubleTopic("Velocity").publish();
        this.maxVelocityPublisher = intakeStateTable.getDoubleTopic("MaxVelocity").publish();
        this.maxAccelerationPublisher = intakeStateTable.getDoubleTopic("MaxAcceleration").publish();
        this.maxAcceleration = maxAcceleration;
        mechanism = new Mechanism2d(1, 4);
        ligament = mechanism
                .getRoot("IntakeRoot", 0.5, 0.0)
                .append(new MechanismLigament2d("Intake", 0.0, 90));
        SmartDashboard.putData(name, mechanism);
    }

    public void telemeterize(IntakeWheelState state) {
        time = Timer.getFPGATimestamp();
        double deltaTime = time - lastTime;
        lastTime = time;
        velocity.set(state.getVelocity().baseUnitMagnitude());
        maxVelocityPublisher.set(maxVelocity.in(MetersPerSecond));
        maxAccelerationPublisher.set(maxAcceleration.in(MetersPerSecondPerSecond));
        timestamp.set(state.getTimestamp().in(Seconds));
        double velocity = state.getVelocity().baseUnitMagnitude() / wheelRadius.baseUnitMagnitude();
        double deltaDegrees = velocity * deltaTime;
        ligament.setAngle(ligamentAngle.mut_plus(deltaDegrees, Degrees).in(Degrees));
    }
}
