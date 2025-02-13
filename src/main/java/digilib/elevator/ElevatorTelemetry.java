package digilib.elevator;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static digilib.DigiMath.roundToDecimal;
import static edu.wpi.first.units.Units.*;

public class ElevatorTelemetry {
    private final DoublePublisher position;
    private final DoublePublisher velocity;
    private final DoublePublisher voltage;
    private final DoublePublisher timestamp;
    private final MechanismLigament2d ligament;

    public ElevatorTelemetry(
            String name,
            Distance minHeight,
            Distance maxHeight,
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        table.getDoubleTopic("MinHeight").publish().set(roundToDecimal(minHeight.baseUnitMagnitude(), 2));
        table.getDoubleTopic("MaxHeight").publish().set(roundToDecimal(maxHeight.baseUnitMagnitude(), 2));
        table.getDoubleTopic("MaxVelocity").publish().set(roundToDecimal(maxVelocity.baseUnitMagnitude(), 2));
        table.getDoubleTopic("MaxAcceleration").publish().set(roundToDecimal(maxAcceleration.baseUnitMagnitude(), 2));
        this.position = table.getDoubleTopic("Position").publish();
        this.velocity = table.getDoubleTopic("Velocity").publish();
        this.voltage = table.getDoubleTopic("Voltage").publish();
        this.timestamp = table.getDoubleTopic("Timestamp").publish();

        Mechanism2d elevatorMechanism = new Mechanism2d(1, 2);
        ligament = elevatorMechanism
                .getRoot("ElevatorRoot", 0.5, 0.0)
                .append(new MechanismLigament2d("Elevator", 0.0, 90));
        SmartDashboard.putData("Elevator", elevatorMechanism);
    }

    public void telemeterize(ElevatorState state) {
        position.set(roundToDecimal(state.getPosition().in(Meters), 2));
        velocity.set(roundToDecimal(state.getVelocity().in(MetersPerSecond), 2));
        voltage.set(roundToDecimal(state.getVoltage().in(Volts), 2));
        timestamp.set(roundToDecimal(state.getTimestamp().in(Seconds), 2));
        ligament.setLength(roundToDecimal(state.getPosition().in(Meters), 2));
    }
}
