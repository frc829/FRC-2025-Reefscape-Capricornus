package frc.robot.mechanisms.elevator;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class ElevatorTelemetry {
    private final NetworkTable elevatorStateTable;
    private final Distance minHeight;
    private final Distance maxHeight;
    private final LinearVelocity maxVelocity;
    private final LinearAcceleration maxAcceleration;
    private final DoublePublisher timestamp;
    private final DoublePublisher height;
    private final DoublePublisher velocity;
    private final DoublePublisher minHeightPublisher;
    private final DoublePublisher maxHeightPublisher;
    private final DoublePublisher maxVelocityPublisher;
    private final DoublePublisher maxAccelerationPublisher;
    
    private final Mechanism2d elevatorMechanism;
    private final MechanismLigament2d elevatorLigament;


    public ElevatorTelemetry(
            String name,
            Distance minHeight,
            Distance maxHeight,
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration) {
        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
        this.maxVelocity = maxVelocity;
        this.elevatorStateTable = NetworkTableInstance.getDefault().getTable(name);
        this.timestamp = elevatorStateTable.getDoubleTopic("Timestamp").publish();
        this.height = elevatorStateTable.getDoubleTopic("Height").publish();
        this.velocity = elevatorStateTable.getDoubleTopic("Velocity").publish();
        this.minHeightPublisher = elevatorStateTable.getDoubleTopic("MinHeightPublisher").publish();
        this.maxHeightPublisher = elevatorStateTable.getDoubleTopic("MaxHeightPublisher").publish();
        this.maxVelocityPublisher = elevatorStateTable.getDoubleTopic("MaxVelocityPublisher").publish();
        this.maxAccelerationPublisher = elevatorStateTable.getDoubleTopic("MaxAccelerationPublisher").publish();
        this.maxAcceleration = maxAcceleration;
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
        minHeightPublisher.set(minHeight.in(Meters));
        maxHeightPublisher.set(maxHeight.in(Meters));
        maxVelocityPublisher.set(maxVelocity.in(MetersPerSecond));
        maxAccelerationPublisher.set(maxAcceleration.in(MetersPerSecondPerSecond));
        // timestamp.set(state.getTimeStamp.in(Seconds));
        height.set(state.getPosition().in(Meters));
        // velocity.set(state.getVelocity.in(MetersPerSecond));
    }
}
