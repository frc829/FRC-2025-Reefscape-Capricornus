package frc.robot.mechanisms.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class KrakenX60Arm extends Arm {

    private final TalonFX talonFX;
    private final CANcoder canCoder;

    public KrakenX60Arm(
            ArmControlParameters armControlParameters,
            TalonFX talonFX,
            CANcoder canCoder) {
        super(armControlParameters);
        this.talonFX = talonFX;
        this.canCoder = canCoder;
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
