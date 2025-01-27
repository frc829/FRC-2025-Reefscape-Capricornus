package frc.robot.mechanisms.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class KrakenX60Arm extends Arm {

    private final TalonFX talonFX;

    public KrakenX60Arm(
            ArmControlParameters armControlParameters,
            TalonFX talonFX) {
        super(armControlParameters);
        this.talonFX = talonFX;
    }

    @Override
    public boolean setNeutralModeToBrake() {
        return false;
    }

    @Override
    public boolean setNeutralModeToCoast() {
        return false;
    }

    @Override
    public boolean setVelocity(AngularVelocity velocity) {
        return false;
    }

    @Override
    public boolean setPosition(Angle position) {
        return false;
    }

    @Override
    public boolean setHold() {
        return false;
    }

    @Override
    public boolean allowFall() {
        return false;
    }

    @Override
    public void resetPosition() {

    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public void updateTelemetry() {

    }
}
