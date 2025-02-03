package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.DistanceUnit;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.derive;

public class RobotConstants {

    private RobotConstants() {
        // utility class
    }

    public static final DistanceUnit Corbins = derive(Feet).aggregate(6).named("Corbin").symbol("cb").make();
    private static final DistanceUnit Corbin = Corbins;
    private static final DistanceUnit Adams = Corbins;
    private static final DistanceUnit Adam = Adams;
    public static final CANBus rio = new CANBus("rio");
    public static final CANBus canivore = new CANBus("canivore", "./logs/example.hoot");


}
