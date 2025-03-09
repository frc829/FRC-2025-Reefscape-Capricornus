package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotController;

public class Constants {

    private Constants() {
        // utility class
    }

    public static final double controllerDeadband = 0.1;
    public static final CANBus rio = new CANBus("rio");
    public static final CANBus canivore = new CANBus("canivore", "./logs/example.hoot");
    public static final String robotComments = RobotController.getComments();




}
