package frc.robot.mechanisms.elevator;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Meters;

public class ElevatorTelemetry {
    // TODO: all fields are private final
    // TODO: create a NetworkTable called elevatorStateTable
    // TODO: create an Distance called minHeight
    // TODO: create an Distance called maxHeight
    // TODO: create an LinearVelocity called maxVelocity
    // TODO: create an LinearAcceleration called maxAcceleration
    // TODO: create a DoublePublisher called timestamp;
    // TODO: the rest are DoublePublishers
    // TODO: for height, velocity, minHeightPublisher, maxHeightPublisher, maxVelocityPublisher, maxAccelerationPublisher,
    private final Mechanism2d elevatorMechanism;
    private final MechanismLigament2d elevatorLigament;

    public ElevatorTelemetry(
            String name,
            Distance minHeight,
            Distance maxHeight,
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration) {
        // TODO: set the fields for minHeight, maxHeight, maxVelocity, maxAcceleration.  Use the this keyword.
        // TODO: this.minHeight = minHeight, etc.
        // TODO: set this.elevatorStateTable to NetworkTableInstance.getDefault().getTable(name)
        // TODO: set this.timestamp to elevatorStateTable.getDoubleTopic("Timestamp").publish();
        // TODO: set this.height to elevatorStateTable.getDoubleTopic("Height").publish()
        // TODO: set this.velocity to elevatorStateTable.getDoubleTopic("Velocity").publish()
        // TODO: repeat for minHeightPublisher, maxHeightPublisher, maxVelocityPublisher, maxAccelerationPublisher
        elevatorMechanism = new Mechanism2d(1, 4);
        elevatorLigament = elevatorMechanism
                .getRoot("ElevatorRoot", 0.5, 0.0)
                .append(new MechanismLigament2d("Elevator", 0.0, 90));
        SmartDashboard.putData("Elevator", elevatorMechanism);
    }

    public void telemeterize(ElevatorState state) {
        // TODO: call minHeightPublisher's set method and pass in minHeight.in(Meters)
        // TODO: repeat for maxHeightPublisher using maxHeight
        // TODO: repeat for maxVelocityPublisher using maxVelocity
        // TODO: repeat for maxAccelerationPublisher using maxAcceleration
        // TODO: repeat for timestamp using state.getTimeStamp.in(Seconds)
        // TODO: repeat for height using state.getPosition.in(Meters)
        // TODO: repeat for velocity using state.getVelocity.in(MetersPerSecond)
        elevatorLigament.setLength(state.getPosition().in(Meters));
    }
}
