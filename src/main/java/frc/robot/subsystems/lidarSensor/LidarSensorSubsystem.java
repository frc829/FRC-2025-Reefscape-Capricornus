package frc.robot.subsystems.lidarSensor;

import com.ctre.phoenix6.Utils;
import digilib.lidarSensor.LidarSensor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LidarSensorSubsystem implements Subsystem {
    private final LidarSensor lidarSensor;
    private final Time simLoopPeriod;

    public LidarSensorSubsystem(
            LidarSensor lidarSensor,
            Time simLoopPeriod) {
        this.lidarSensor = lidarSensor;
        this.simLoopPeriod = simLoopPeriod;
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Trigger gte(double distanceMM) {
        return new Trigger(() -> lidarSensor
                .getState()
                .getDistanceMeters() * 1000.0 >= distanceMM);
    }

    public Trigger lte(double distanceMM) {
        return new Trigger(() -> lidarSensor
                .getState()
                .getDistanceMeters() * 1000.0 <= distanceMM);
    }

    public Trigger inRange(double minDistanceMM, double maxDistanceMM) {
        return gte(minDistanceMM).and(lte(maxDistanceMM));
    }

    @Override
    public void periodic() {
        lidarSensor.update();
    }

    @SuppressWarnings("resource")
    private void startSimThread() {
        new Notifier(lidarSensor::updateSimState)
                .startPeriodic(simLoopPeriod.baseUnitMagnitude());
    }
}
