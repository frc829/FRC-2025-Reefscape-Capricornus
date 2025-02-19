package digilib.intakeWheel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static digilib.DigiMath.roundToDecimal;

public class IntakeWheelTelemetry {
    private final DoublePublisher velocity;
    private final DoublePublisher voltage;
    private final DoublePublisher timestamp;
    private final MechanismLigament2d ligament;
    private final DoublePublisher current;


    public IntakeWheelTelemetry(
            String name,
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getDoubleTopic("Max Velocity").publish().set(maxVelocity.baseUnitMagnitude());
        table.getDoubleTopic("Max Acceleration").publish().set(maxAcceleration.baseUnitMagnitude());
        velocity = table.getDoubleTopic("Velocity").publish();
        voltage = table.getDoubleTopic("Voltage").publish();
        current = table.getDoubleTopic("Current").publish();
        timestamp = table.getDoubleTopic("Timestamp").publish();

        Mechanism2d mechanism = new Mechanism2d(2, 4);
        ligament = mechanism
                .getRoot("IntakeRoot", 1, 3)
                .append(new MechanismLigament2d("Intake", 0.0, 90, 1, new Color8Bit(Color.kRed)));
        SmartDashboard.putData(name, mechanism);
    }

    public void telemeterize(IntakeWheelState state) {
        velocity.set(roundToDecimal(state.getAngularVelocity().baseUnitMagnitude(), 2));
        voltage.set(roundToDecimal(state.getVoltage().baseUnitMagnitude(), 2));
        timestamp.set(roundToDecimal(state.getTimestamp().baseUnitMagnitude(), 2));
        ligament.setLength(roundToDecimal(state.getAngularVelocity().baseUnitMagnitude(), 2));
        current.set(roundToDecimal(state.getCurrent().baseUnitMagnitude(), 2));
    }
}
