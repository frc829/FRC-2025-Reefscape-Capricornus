package frc.robot.subsystems.hook;

import digilib.arm.Arm;
import edu.wpi.first.units.measure.Time;

public class CommandHook {

    private final Arm arm;
    private final Time simLoopPeriod;

    public CommandHook(Arm arm, Time simLoopPeriod) {
        this.arm = arm;
        this.simLoopPeriod = simLoopPeriod;
    }

}
