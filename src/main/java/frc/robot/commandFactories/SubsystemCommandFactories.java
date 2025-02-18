package frc.robot.commandFactories;

import digilib.arm.ArmRequest;
import digilib.intakeWheel.IntakeWheelRequest;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.dualIntake.DualIntakeSubsystem;
import frc.robot.subsystems.pneumatics.ClawSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pneumatics.PneumaticSubsystem;
import frc.robot.subsystems.power.PowerSubsystem;
import frc.robot.subsystems.swerveDrive.CommandSwerveDriveFactory;
import frc.robot.subsystems.wrist.WristSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class SubsystemCommandFactories {
    private final ClawSubsystem algae;
    private final ArmSubsystem arm;
    private final ClawSubsystem coral;
    private final DualIntakeSubsystem dualIntake;
    private final ElevatorSubsystem elevator;
    private final PneumaticSubsystem pneumatics;
    private final PowerSubsystem power;
    private final CommandSwerveDriveFactory swerve;
    // public final WinchSubsystem winch;
    private final WristSubsystem wrist;
    public final Trigger hasAlgae;
    public final Trigger hasCoral;

    public SubsystemCommandFactories(ClawSubsystem algae, ArmSubsystem arm, ClawSubsystem coral, DualIntakeSubsystem dualIntake, ElevatorSubsystem elevator, PneumaticSubsystem pneumatics, PowerSubsystem power, CommandSwerveDriveFactory commandSwerveDriveFactory,
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
        this.hasAlgae = dualIntake.hasAlgae;
        this.hasCoral = dualIntake.hasCoral;

        SmartDashboard.putData("Power: Clear Sticky Faults", power.clearFaults());
        SmartDashboard.putData("Pneumatics: Clear Sticky Faults", pneumatics.clearFaults());
    }

    public Trigger armAtAngle(Angle angle, Angle tolerance) {
        return arm.atPosition(angle, tolerance);
    }

    public Trigger elevatorAtHeight(Distance height, Distance tolerance) {
        return elevator.atPosition(height, tolerance);
    }

    public Trigger wristAtAngle(Angle angle, Angle tolerance) {
        return wrist.atPosition(angle, tolerance);
    }

    public Command elevatorTo(Distance height, Distance tolerance) {
        return elevator.goToPosition(height, tolerance).asProxy();
    }

    public Command armTo(Angle angle, Angle tolerance) {
        ArmRequest.Position request = new ArmRequest.Position();
        request.withPosition(angle.in(Radians));
        return arm.applyRequest(() -> request)
                .until(arm.atPosition(angle, tolerance))
                .withName(String.format("%s: %s deg, %s deg tolerance", arm.getName(), angle.in(Degrees), tolerance.in(Degrees)));
    }

    public Command armAtSpeed(DoubleSupplier value) {
        ArmRequest.Velocity request = new ArmRequest.Velocity();
        return arm.applyRequest(() -> request.withVelocity(value.getAsDouble()))
                .withName(String.format("%s: VELOCITY", arm.getName()));
    }

    public Command intakeSpin(DoubleSupplier algaeSpeed, DoubleSupplier coralSpeed) {
        IntakeWheelRequest.Velocity request0 = new IntakeWheelRequest.Velocity();
        IntakeWheelRequest.Velocity request1 = new IntakeWheelRequest.Velocity();
        Pair<IntakeWheelRequest, IntakeWheelRequest> intakeRequests = new Pair<>(request0, request1);

        return dualIntake.applyRequest(() -> {
                    request0.withVelocity(algaeSpeed.getAsDouble());
                    request1.withVelocity(coralSpeed.getAsDouble());
                    return intakeRequests;
                }).asProxy()
                .withName(String.format("%s: VELOCITY", dualIntake.getName()));
    }

    public Command closeAlgaeClaw() {
        return algae.close().asProxy();
    }

    public Command closeCoralClaw() {
        return coral.close().asProxy();
    }

    public Command openAlgaeClaw() {
        return algae.open().asProxy();
    }

    public Command openCoralClaw() {
        return coral.open().asProxy();
    }
}
