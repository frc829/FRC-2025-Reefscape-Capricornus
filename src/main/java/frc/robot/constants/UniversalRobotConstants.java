package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class UniversalRobotConstants {

    private UniversalRobotConstants() {
        // utility class
    }

    public static final CANBus rio = new CANBus("rio");
    public static final CANBus canivore = new CANBus("canivore", "./logs/example.hoot");


}
