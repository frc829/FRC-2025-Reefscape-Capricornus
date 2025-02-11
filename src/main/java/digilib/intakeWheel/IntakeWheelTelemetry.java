package digilib.intakeWheel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static digilib.DigiMath.roundToDecimal;
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
    private final Distance wheelRadius;


    public IntakeWheelTelemetry(
            String name,
            Distance wheelRadius,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        this.wheelRadius = wheelRadius;
        this.maxVelocity = MetersPerSecond.of(maxVelocity.baseUnitMagnitude() * wheelRadius.baseUnitMagnitude());
        this.intakeStateTable = NetworkTableInstance.getDefault().getTable(name);
        this.timestamp = intakeStateTable.getDoubleTopic("Timestamp").publish();
        this.velocity = intakeStateTable.getDoubleTopic("Velocity").publish();
        this.maxVelocityPublisher = intakeStateTable.getDoubleTopic("MaxVelocity").publish();
        this.maxAccelerationPublisher = intakeStateTable.getDoubleTopic("MaxAcceleration").publish();
        this.maxAcceleration = MetersPerSecondPerSecond.of(maxAcceleration.baseUnitMagnitude() * wheelRadius.baseUnitMagnitude());
        mechanism = new Mechanism2d(2, 4);
        ligament = mechanism
                .getRoot("IntakeRoot", 1, 3)
                .append(new MechanismLigament2d("Intake", 0.0, 90, 1, new Color8Bit(Color.kRed)));
        SmartDashboard.putData(name, mechanism);
    }

    public void telemeterize(IntakeWheelState state) {
        velocity.set(roundToDecimal(state.getVelocity().baseUnitMagnitude(), 2));
        maxVelocityPublisher.set(roundToDecimal(maxVelocity.in(MetersPerSecond), 2));
        maxAccelerationPublisher.set(roundToDecimal(maxAcceleration.in(MetersPerSecondPerSecond), 2));
        timestamp.set(roundToDecimal(state.getTimestamp().in(Seconds), 2));
        double velocity = roundToDecimal(state.getVelocity().baseUnitMagnitude() / wheelRadius.baseUnitMagnitude(), 2);
        ligament.setLength(velocity / maxVelocity.baseUnitMagnitude());
    }
}
