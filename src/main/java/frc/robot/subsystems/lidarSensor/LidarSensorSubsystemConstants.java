package frc.robot.subsystems.lidarSensor;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import digilib.lidarSensor.LaserCanLidarSensor;
import digilib.lidarSensor.LidarSensor;
import edu.wpi.first.wpilibj.Alert;

import static digilib.lidarSensor.LidarSensor.*;
import static edu.wpi.first.wpilibj.Alert.AlertType.*;
import static frc.robot.subsystems.lidarSensor.LidarSensorSubsystemConstants.Device.*;
import static frc.robot.subsystems.lidarSensor.LidarSensorSubsystemConstants.Sensor.*;

public class LidarSensorSubsystemConstants {

    static final class Device {
        static final String name = "Coral Detector";
        static final Config config = new Config(name);
    }

    static final class Sensor {
        static final int laserCanId = 36;
        static final LaserCanInterface.RangingMode rangingMode = LaserCanInterface.RangingMode.SHORT;
        static final LaserCanInterface.RegionOfInterest regionOfInterest = new LaserCanInterface.RegionOfInterest(8, 8, 16, 16);
        static final LaserCanInterface.TimingBudget timingBudget = LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS;
    }


    public static LidarSensorSubsystem create() {
        LaserCan laserCan;
        try {
            laserCan = new LaserCan(laserCanId);
            laserCan.setRangingMode(rangingMode);
            laserCan.setRegionOfInterest(regionOfInterest);
            laserCan.setTimingBudget(timingBudget);
        } catch (Exception e) {
            laserCan = null;
            //noinspection resource
            Alert alert = new Alert(e.getMessage(), kError);
            alert.set(true);
        }
        LidarSensor lidarSensor = new LaserCanLidarSensor(
                config,
                laserCan);
        LidarSensorSubsystem subsystem = new LidarSensorSubsystem(lidarSensor);
        subsystem.register();
        return subsystem;
    }
}
