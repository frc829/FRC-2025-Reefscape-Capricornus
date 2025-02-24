package digilib.swerve;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public record SwerveDriveConstants(
        LinearVelocity maxVelocity,
        AngularVelocity maxAngularVelocity,
        PhoenixPIDController pathXController,
        PhoenixPIDController pathYController,
        PhoenixPIDController pathThetaController) {
}
