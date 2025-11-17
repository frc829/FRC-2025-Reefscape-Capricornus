package digilib.swerve;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveModule;


public class CTRESwerveModule<
        DriveMotorT extends CommonTalon,
        SteerMotorT extends CommonTalon,
        EncoderT extends ParentDevice> implements Module {

    private final SwerveModule<DriveMotorT,
            SteerMotorT,
            EncoderT> module;

    public CTRESwerveModule(SwerveModule<DriveMotorT,
            SteerMotorT,
            EncoderT> module) {

        this.module = module;
    }
}
