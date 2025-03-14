package digilib.elevator;

public class ElevatorState {

    private double motorEncoderPositionMeters = 0.0;
    private double motorEncoderVelocityMPS = 0.0;
    private double volts = 0.0;
    private double amps = 0.0;
    private double positionSetpointMeters = 0.0;
    private double velocitySetpointMPS = 0.0;

    public double getMotorEncoderPositionMeters() {
        return motorEncoderPositionMeters;
    }

    public void setMotorEncoderPositionMeters(double motorEncoderPositionMeters) {
        this.motorEncoderPositionMeters = motorEncoderPositionMeters;
    }

    public double getMotorEncoderVelocityMPS() {
        return motorEncoderVelocityMPS;
    }

    public void setMotorEncoderVelocityMPS(double motorEncoderVelocityMPS) {
        this.motorEncoderVelocityMPS = motorEncoderVelocityMPS;
    }

    public double getVolts() {
        return volts;
    }

    public void setVolts(double volts) {
        this.volts = volts;
    }

    public double getAmps() {
        return amps;
    }

    public void setAmps(double amps) {
        this.amps = amps;
    }

    public double getPositionSetpointMeters() {
        return positionSetpointMeters;
    }

    public void setPositionSetpointMeters(double positionSetpointMeters) {
        this.positionSetpointMeters = positionSetpointMeters;
    }

    public double getVelocitySetpointMPS() {
        return velocitySetpointMPS;
    }

    public void setVelocitySetpointMPS(double velocitySetpointMPS) {
        this.velocitySetpointMPS = velocitySetpointMPS;
    }
}
