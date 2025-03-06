package digilib.swerve;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

public record SwerveDriveConstants(
        String name,
        double maxVelocityMPS,
        double maxAngularVelocityRPS,
        double maxVelocityDeadbandScalar,
        double maxAngularVelocityDeadbandScalar,
        PhoenixPIDController pathXController,
        PhoenixPIDController pathYController,
        PhoenixPIDController pathThetaController) {
}
