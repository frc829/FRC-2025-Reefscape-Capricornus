package digilib.elevator;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public record ElevatorConstants(String name,
                                double reduction,
                                Distance drumRadius,
                                Distance maxHeight,
                                Distance minHeight,
                                Distance startingHeight,
                                Voltage ks,
                                Voltage kg,
                                Measure<? extends PerUnit<VoltageUnit, LinearVelocityUnit>> kv,
                                Measure<? extends PerUnit<VoltageUnit, LinearAccelerationUnit>> ka,
                                LinearVelocity maxVelocity,
                                LinearAcceleration maxAcceleration,
                                Distance positionStdDev,
                                LinearVelocity velocityStdDev) {
}
