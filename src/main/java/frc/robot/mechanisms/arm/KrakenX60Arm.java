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
    public void setVelocity(AngularVelocity velocity) {
    }

    @Override
    public void setPosition(Angle position) {
    }

    @Override
    public void setHold() {
    }

    @Override
    public void setFreeFall() {
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

    private void applyVelocity(){

    }

    private void applyPosition(){

    }

    private void applyFreeFall(){
    }
}
