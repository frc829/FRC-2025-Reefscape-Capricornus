package digilib.arm;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public record ArmConstants(String name,
                           double reduction,
                           Angle maxAngle,
                           Angle minAngle,
                           Angle startingAngle,
                           Voltage ks,
                           Voltage kg,
                           Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv,
                           Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka,
                           AngularVelocity maxAngularVelocity,
                           AngularAcceleration maxAngularAcceleration,
                           Angle positionStdDev,
                           AngularVelocity velocityStdDev) {
}
