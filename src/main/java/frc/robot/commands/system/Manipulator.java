package frc.robot.commands.system;

import digilib.claws.ClawRequest;
import digilib.claws.ClawValue;
import digilib.elevator.ElevatorRequest;
import digilib.intakeWheel.IntakeWheelRequest;
import digilib.wrist.WristRequest;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.dualIntake.DualIntakeSubsystem;
import frc.robot.subsystems.pneumatics.ClawSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pneumatics.PneumaticSubsystem;
import frc.robot.subsystems.power.PowerSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Manipulator {
    private final ClawSubsystem algae;
    private final ArmSubsystem arm;
    private final ClawSubsystem coral;
    private final DualIntakeSubsystem dualIntake;
    private final ElevatorSubsystem elevator;
    private final PneumaticSubsystem pneumatics;
    private final PowerSubsystem power;
    private final WristSubsystem wrist;
    public final Trigger hasAlgae;
    public final Trigger hasCoral;
    public final Trigger isCoralClawClosed;
    public final Trigger isAlgaeClawClosed;

    public Manipulator(
            ClawSubsystem algae,
            ArmSubsystem arm,
            ClawSubsystem coral,
            DualIntakeSubsystem dualIntake,
            ElevatorSubsystem elevator,
            PneumaticSubsystem pneumatics,
            PowerSubsystem power,
            WristSubsystem wrist) {
        this.algae = algae;
        this.arm = arm;
        this.coral = coral;
        this.dualIntake = dualIntake;
        this.elevator = elevator;
        this.pneumatics = pneumatics;
        this.power = power;
        this.wrist = wrist;
        this.hasAlgae = dualIntake.hasAlgae;
        this.hasCoral = dualIntake.hasCoral;
        this.isCoralClawClosed = coral.isClosed;
        this.isAlgaeClawClosed = algae.isClosed;

        SmartDashboard.putData("Power: Clear Sticky Faults", power.clearFaults());
        SmartDashboard.putData("Pneumatics: Clear Sticky Faults", pneumatics.clearFaults());
    }

    public Trigger armAtAngle(Angle angle, Angle tolerance) {
        return arm.atPosition(angle, tolerance);
    }

    public Trigger armGreaterThan(Angle angle) {
        return arm.greaterThan(angle);
    }

    public Trigger armLessThan(Angle angle) {
        return arm.lessThan(angle);
    }

    public Trigger elevatorGreaterThan(Distance distance) {
        return elevator.greaterThan(distance);
    }

    public Trigger elevatorLessThan(Distance distance) {
        return elevator.lessThan(distance);
    }

    public Trigger wristGreaterThan(Angle angle) {
        return wrist.greaterThan(angle);
    }

    public Trigger wristLessThan(Angle angle) {
        return wrist.lessThan(angle);
    }

    public Trigger elevatorAtHeight(Distance height, Distance tolerance) {
        return elevator.atPosition(height, tolerance);
    }

    public Trigger wristAtAngle(Angle angle, Angle tolerance) {
        return wrist.atPosition(angle, tolerance);
    }

    public Command elevatorTo(Distance height) {
        ElevatorRequest.Position request = new ElevatorRequest.Position();
        return elevator.applyRequest(() -> request.withPosition(height))
                .withName(String.format(String.format("%s: %s meters", elevator.getName(), height.in(Meters))));
    }

    public Command armTo(Angle angle) {
        ArmRequest.Position request = new ArmRequest.Position();
        return arm.applyRequest(() -> request.withPosition(angle))
                .withName(String.format("%s: %s deg", arm.getName(), angle.in(Degrees)));
    }

    public Command wristTo(Angle angle) {
        WristRequest.Position request = new WristRequest.Position();
        return wrist.applyRequest(() -> request.withAngle(angle))
                .withName(String.format("%s: %s deg", wrist.getName(), angle.in(Degrees)));
    }

    public Command armToSpeed(Supplier<Dimensionless> maxPercent) {
        ArmRequest.Velocity request = new ArmRequest.Velocity();
        return arm.applyRequest(() -> request.withVelocity(maxPercent.get()))
                .withName(String.format("%s: VELOCITY", arm.getName()));
    }

    public Command elevatorToSpeed(Supplier<Dimensionless> maxPercent) {
        ElevatorRequest.Velocity request = new ElevatorRequest.Velocity();
        return elevator.applyRequest(() -> request.withVelocity(maxPercent.get()))
                .withName(String.format("%s: VELOCITY", elevator.getName()));
    }

    public Command wristToSpeed(Supplier<Dimensionless> maxPercent) {
        WristRequest.Velocity request = new WristRequest.Velocity();
        return wrist.applyRequest(() -> request.withVelocity(maxPercent.get()))
                .withName(String.format("%s: VELOCITY", wrist.getName()));
    }

    public Command intakeToSpeed(Dimensionless wheel0MaxPercent, Dimensionless wheel1MaxPercent) {
        IntakeWheelRequest.Velocity request0 = new IntakeWheelRequest.Velocity();
        IntakeWheelRequest.Velocity request1 = new IntakeWheelRequest.Velocity();
        Pair<IntakeWheelRequest, IntakeWheelRequest> intakeRequests = new Pair<>(request0, request1);

        return dualIntake.applyRequest(() -> {
                    request0.withVelocity(wheel0MaxPercent);
                    request1.withVelocity(wheel1MaxPercent);
                    return intakeRequests;
                })
                .withName(String.format("%s: VELOCITY", dualIntake.getName()));
    }

    public Command intakeAtVolts(Voltage volts0, Voltage volts1) {
        IntakeWheelRequest.VoltageRequest request0 = new IntakeWheelRequest.VoltageRequest();
        IntakeWheelRequest.VoltageRequest request1 = new IntakeWheelRequest.VoltageRequest();
        Pair<IntakeWheelRequest, IntakeWheelRequest> intakeRequests = new Pair<>(request0, request1);

        return dualIntake.applyRequest(() -> {
                    request0.withVoltage(volts0);
                    request1.withVoltage(volts1);
                    return intakeRequests;
                })
                .withName(String.format("%s: VOLTAGE", dualIntake.getName()));
    }

    public Command setAlgaeClaw(ClawValue value) {
        ClawRequest.SetValue request = new ClawRequest.SetValue();
        return algae.applyRequestOnce(() -> request.withClawValue(value))
                .withName(String.format("%s: %s", algae.getName(), value.toString().toUpperCase()));
    }

    public Command setCoralClaw(ClawValue value) {
        ClawRequest.SetValue request = new ClawRequest.SetValue();
        return coral.applyRequestOnce(() -> request.withClawValue(value))
                .withName(String.format("%s: %s", coral.getName(), value.toString().toUpperCase()));
    }

    public Command toggleAlgaeClaw() {
        ClawRequest.Toggle request = new ClawRequest.Toggle();
        return algae.applyRequestOnce(() -> request)
                .withName(String.format("%s: TOGGLE", algae.getName()));
    }

    public Command toggleCoralClaw() {
        ClawRequest.Toggle request = new ClawRequest.Toggle();
        return coral.applyRequestOnce(() -> request)
                .withName(String.format("%s: TOGGLE", coral.getName()));
    }
}
