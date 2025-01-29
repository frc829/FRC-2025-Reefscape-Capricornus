package frc.robot.mechanisms.intake;

import edu.wpi.first.units.measure.LinearVelocity;

public abstract class DualIntake {

    public enum ControlState{
        VELOCITY,
        IDLE
    }


    protected final DualIntakeControlParameters dualIntakeControlParameters;
    protected final DualIntakeState lastDualIntakeState = new DualIntakeState();
    protected final DualIntakeState dualIntakeState = new DualIntakeState();
    protected final DualSimIntake dualSimIntake = new DualSimIntake();
    private DualIntakeRequest dualIntakeRequest = new DualIntakeRequest.Idle();

    public DualIntake(DualIntakeControlParameters dualIntakeControlParameters) {
        this.dualIntakeControlParameters = dualIntakeControlParameters;
    }

    public void updateSimState(double dtSeconds, double supplyVoltage){
        dualSimIntake.update(dtSeconds, supplyVoltage);
    }

    public void setControl(DualIntakeRequest request){
        if(dualIntakeRequest != request){
            dualIntakeRequest = request;
        }
        dualIntakeControlParameters.withIntakeState(dualIntakeState);
        request.apply(dualIntakeControlParameters, this);
    }

    public final DualIntakeState getState(){
        return dualIntakeState;
    }

    public final DualIntakeState getStateCopy(){
        return dualIntakeState.clone();
    }

    public DualIntakeState getLastIntakeState() {
        return lastDualIntakeState;
    }

    public abstract void updateTelemetry();

    public abstract boolean setNeutralModeToBrake();

    public abstract boolean setNeutralModeToCoast();

    public abstract void setVelocity(LinearVelocity intake0Velocity, LinearVelocity intake1Velocity);

    public abstract void setIdle();

    public void update(){
        updateTelemetry();
    }



}
