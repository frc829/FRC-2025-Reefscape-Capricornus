package digilib.intakeWheel;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public record IntakeWheelConstants(String name,
                                   double reduction,
                                   Distance wheelRadius,
                                   Voltage ks,
                                   Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kv,
                                   Measure<? extends PerUnit<VoltageUnit, AngularAccelerationUnit>> ka,
                                   AngularVelocity maxVelocity, AngularAcceleration maxAcceleration,
                                   AngularVelocity velocityStdDev) {
}
