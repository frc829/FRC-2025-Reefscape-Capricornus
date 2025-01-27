package frc.robot.mechanisms.intake;

import edu.wpi.first.units.measure.LinearVelocity;

public abstract class Intake {

    public enum ControlState{
        VELOCITY,
        IDLE
    }

    protected final IntakeControlParameters intakeControlParameters;
    protected final IntakeState lastIntakeState = new IntakeState();
    protected final IntakeState intakeState = new IntakeState();
    protected final SimIntake simIntake = new SimIntake();
    private IntakeRequest intakeRequest = new IntakeRequest.Idle();

    public Intake(IntakeControlParameters intakeControlParameters) {
        this.intakeControlParameters = intakeControlParameters;
    }

    public void updateSimState(double dtSeconds, double supplyVoltage){
        simIntake.update(dtSeconds, supplyVoltage);
    }

    public void setControl(IntakeRequest request){
        if(intakeRequest != request){
            intakeRequest = request;
        }
        intakeControlParameters.withIntakeState(intakeState);
        request.apply(intakeControlParameters, this);
    }

    public final IntakeState getState(){
        return intakeState;
    }

    public final IntakeState getStateCopy(){
        return intakeState.clone();
    }

    public IntakeState getLastIntakeState() {
        return lastIntakeState;
    }

    public abstract void updateTelemetry();

    public abstract boolean setNeutralModeToBrake();

    public abstract boolean setNeutralModeToCoast();

    public abstract void setVelocity(LinearVelocity velocity);

    public abstract void setIdle();

    public void update(){
        updateTelemetry();
    }



}
