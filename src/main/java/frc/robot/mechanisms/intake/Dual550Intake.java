package frc.robot.mechanisms.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.measure.LinearVelocity;

public class Dual550Intake extends Intake {

    private final SparkMax topMotor;
    private final SparkMax bottomMotor;
    private final SparkBaseConfig topMotorConfig;
    private final SparkBaseConfig bottomMotorConfig;

    public Dual550Intake(
            IntakeControlParameters intakeControlParameters,
            SparkMax topMotor,
            SparkMax bottomMotor,
            SparkBaseConfig topMotorConfig,
            SparkBaseConfig bottomMotorConfig) {
        super(intakeControlParameters);
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
        this.topMotorConfig = topMotorConfig;
        this.bottomMotorConfig = bottomMotorConfig;
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
    public void setVelocity(LinearVelocity velocity) {
    }

    @Override
    public void setIdle() {
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

    private void applyIdle(){
    }
}
