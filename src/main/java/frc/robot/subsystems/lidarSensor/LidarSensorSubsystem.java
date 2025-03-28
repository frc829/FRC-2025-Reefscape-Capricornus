package frc.robot.subsystems.lidarSensor;

import digilib.lidarSensor.LidarSensor;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LidarSensorSubsystem implements Subsystem {
    private final LidarSensor lidarSensor;

    public LidarSensorSubsystem(
            LidarSensor lidarSensor) {
        this.lidarSensor = lidarSensor;
    }

    public Trigger gte(double distanceMM) {
        return new Trigger(() -> lidarSensor
                .getDistanceMillimeters() >= distanceMM);
    }

    public Trigger lte(double distanceMM) {
        return new Trigger(() -> lidarSensor
                .getDistanceMillimeters() <= distanceMM);
    }

    public Trigger inRange(double minDistanceMM, double maxDistanceMM) {
        return gte(minDistanceMM).and(lte(maxDistanceMM));
    }

    @Override
    public void periodic() {
        lidarSensor.update();
    }
}
