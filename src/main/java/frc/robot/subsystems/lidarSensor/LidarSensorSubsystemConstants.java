package frc.robot.subsystems.lidarSensor;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import digilib.lidarSensor.LaserCanLidarSensor;
import digilib.lidarSensor.LidarSensor;
import digilib.lidarSensor.LidarSensorConstants;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.IntSupplier;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj.Alert.AlertType.*;
import static frc.robot.subsystems.lidarSensor.LidarSensorSubsystemConstants.Sensor.*;
import static frc.robot.subsystems.lidarSensor.LidarSensorSubsystemConstants.Simulation.*;

public class LidarSensorSubsystemConstants {

    static final class Simulation {
        static final CommandXboxController controller = new CommandXboxController(5);
        static final IntSupplier simulationDistanceMM = () -> {
            if (controller.a().getAsBoolean()) {
                return 0;
            } else {
                return Integer.MAX_VALUE;
            }
        };
        static final Time simLoopPeriod = Seconds.of(0.001);
    }

    static final class Device {
        static final String name = "Coral Detector";
        static final LidarSensorConstants constants = new LidarSensorConstants(name, simulationDistanceMM);
    }

    static final class Sensor {
        static final int laserCanId = 36;
        static final RangingMode rangingMode = RangingMode.LONG;
        static final RegionOfInterest regionOfInterest = new RegionOfInterest(0, 0, 16, 16);
        static final LaserCanInterface.TimingBudget timingBudget = LaserCanInterface.TimingBudget.TIMING_BUDGET_100MS;
    }


    public static LidarSensorSubsystem create() {
        LaserCan laserCan;
        try {
            laserCan = new LaserCan(laserCanId);
            laserCan.setRangingMode(rangingMode);
            laserCan.setRegionOfInterest(regionOfInterest);
            laserCan.setTimingBudget(timingBudget);
        } catch (ConfigurationFailedException e) {
            laserCan = null;
            Alert alert = new Alert(e.getMessage(), kError);
            alert.set(true);
        }
        LidarSensor lidarSensor = new LaserCanLidarSensor(
                Device.constants,
                laserCan);
        return new LidarSensorSubsystem(lidarSensor, simLoopPeriod);
    }
}
