package frc.robot.commandFactories;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.dualIntake.DualIntakeSubsystem;
import frc.robot.subsystems.pneumatics.ClawSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pneumatics.PneumaticSubsystem;
import frc.robot.subsystems.power.PowerSubsystem;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveFactory;
import frc.robot.subsystems.winch.WinchSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class SubsystemCommandFactories {
    public final ClawSubsystem algae;
    public final ArmSubsystem arm;
    public final ClawSubsystem coral;
    public final DualIntakeSubsystem dualIntake;
    public final ElevatorSubsystem elevator;
    public final PneumaticSubsystem pneumatics;
    public final PowerSubsystem power;
    public final CommandSwerveDriveFactory swerve;
    // public final WinchSubsystem winch;
    public final WristSubsystem wrist;


    public SubsystemCommandFactories(
            ClawSubsystem algae,
            ArmSubsystem arm,
            ClawSubsystem coral,
            DualIntakeSubsystem dualIntake,
            ElevatorSubsystem elevator,
            PneumaticSubsystem pneumatics,
            PowerSubsystem power,
            CommandSwerveDriveFactory commandSwerveDriveFactory,
            // WinchSubsystem winch){
            WristSubsystem wrist) {
        this.algae = algae;
        this.arm = arm;
        this.coral = coral;
        this.dualIntake = dualIntake;
        this.elevator = elevator;
        this.pneumatics = pneumatics;
        this.power = power;
        this.swerve = commandSwerveDriveFactory;
        // this.winch = winch;
        this.wrist = wrist;
        SmartDashboard.putData("Power: Clear Sticky Faults", power.clearFaults());
        SmartDashboard.putData("Pneumatics: Clear Sticky Faults", pneumatics.clearFaults());
    }
}
