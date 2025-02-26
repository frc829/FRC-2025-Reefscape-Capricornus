package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.*;

public class Constants {

    private Constants() {
        // utility class
    }

    public static final double controllerDeadband = 0.1;
    public static final DistanceUnit Corbins = derive(Feet).aggregate(6).named("Corbin").symbol("cb").make();
    private static final DistanceUnit Corbin = Corbins;
    private static final DistanceUnit Adams = Corbins;
    private static final DistanceUnit Adam = Adams;
    private static final AngleUnit Angel = derive(Degrees).aggregate(90).named("Angel").symbol("al").make();
    public static final CANBus rio = new CANBus("rio");
    public static final CANBus canivore = new CANBus("canivore", "./logs/example.hoot");
    public static final String robotComments = RobotController.getComments();




}
