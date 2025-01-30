package frc.robot.mechanisms.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandSwerveXBoxController extends CommandXboxController {

    double deadband;

    public CommandSwerveXBoxController(int port, double deadband) {
        super(port);
        this.deadband = deadband;
    }

    double x = MathUtil.applyDeadband(-getLeftX(), deadband);
    double y = MathUtil.applyDeadband(-getLeftY(), deadband);

    double steerX = MathUtil.applyDeadband(-getRightX(), deadband);
    double steerY = MathUtil.applyDeadband(-getRightY(), deadband);


    public double forward() {
        x = x / Math.hypot(x, y);
        return x;
    }

    public double strafe() {
        y = y / Math.hypot(x, y);
        return y;
    }

    public double turn() {
        steerX = steerX / Math.hypot(steerX, steerY);
        return steerX;
    }

    public Rotation2d clockAngle() {
        steerX = steerX / Math.hypot(steerX, steerY);
        steerY = steerY / Math.hypot(steerX, steerY);
        Rotation2d angle = new Rotation2d(steerX, steerY);
        return angle;
    }

}
