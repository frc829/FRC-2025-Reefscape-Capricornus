package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Manipulator;
import frc.robot.subsystems.swerveDrive.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.either;

public class Manual {

    private final SwerveDriveSubsystem swerveDrive;
    private final Manipulator manipulator;
    public final Trigger hasAlgae;
    public final Trigger hasCoral;

    public Manual(SwerveDriveSubsystem swerveDrive, Manipulator manipulator) {
        this.swerveDrive = swerveDrive;
        this.manipulator = manipulator;
        hasAlgae = manipulator.hasAlgae();
        hasCoral = manipulator.hasCoral();
    }

    public Command manualArm(DoubleSupplier setpointScalar) {
        return manipulator.arm().toVelocity(setpointScalar);
    }

    public Command manualElevator(DoubleSupplier setpointScalar) {
        return manipulator.elevator().toVelocity(setpointScalar);
    }

    public Command manualElevatorDangerous(DoubleSupplier setpointScalar){
        return manipulator.elevator().toVoltage(setpointScalar);
    }

    public Command manualWristToggle() {
        return either(
                manipulator.wrist().toAngle(90.0),
                manipulator.wrist().toAngle(0.0),
                manipulator.wrist().inRange(-10.0, 45.0))
                .withName("Manual Wrist Toggle");
    }

    public Command manualWrist(DoubleSupplier setpointScalar) {
        return manipulator.wrist().toVelocity(setpointScalar);
    }

    public Command manualAlgaeIntake(DoubleSupplier setpointScalar) {
        return manipulator.algaeIntakeWheel().toVelocity(setpointScalar);
    }

    public Command manualCoralIntake(DoubleSupplier setpointScalar) {
        return manipulator.coralIntakeWheel().toVelocity(setpointScalar);
    }

    public Command manualAlgaeClawToggle() {
        return manipulator.algaeClaw().toggle();
    }

    public Command manualCoralClawToggle() {
        return manipulator.coralClaw().toggle();
    }

    public Command manualElevatorTest() {
        return manipulator.elevator().toHeight(60.0 / 100.0);
    }


    public Command manualElevatorTestDown() {
        return manipulator.elevator().toHeight(10.0 / 100.0);
    }

    public Command manualArm60Test() {
        return manipulator.arm().toAngle(60.0);
    }

    public Command manualArm0Test(){
        return manipulator.arm().toAngle(0.0);
    }

    public Command manualSteer90Test(){
        return swerveDrive.pointSteer(() -> 90);
    }

    public Command manualSteer0Test(){
        return swerveDrive.pointSteer(() -> 0);
    }


}
