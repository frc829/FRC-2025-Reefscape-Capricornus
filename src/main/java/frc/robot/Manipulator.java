package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intakeWheel.IntakeWheelSubsystem;
import frc.robot.subsystems.lidarSensor.LidarSensorSubsystem;
import frc.robot.subsystems.pneumatics.ClawSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public record Manipulator(
        ArmSubsystem arm,
        ClawSubsystem algaeClaw,
        ClawSubsystem coralClaw,
        ElevatorSubsystem elevator,
        IntakeWheelSubsystem coralIntakeWheel,
        LidarSensorSubsystem lidarSensor,
        WristSubsystem wrist) {

    public Trigger hasCoral() {
        return lidarSensor().inRange(1, 7);
    }

}
