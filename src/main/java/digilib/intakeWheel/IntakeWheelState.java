package digilib.intakeWheel;

public class IntakeWheelState {

    private double motorEncoderVelocityRPS = 0.0;
    private double volts = 0.0;
    private double amps = 0.0;

    public double getMotorEncoderVelocityRPS() {
        return motorEncoderVelocityRPS;
    }

    public double getMotorEncoderVelocityDPS() {
        return motorEncoderVelocityRPS * 360.0;
    }

    public void setMotorEncoderVelocityRPS(double motorEncoderVelocityRPS) {
        this.motorEncoderVelocityRPS = motorEncoderVelocityRPS;
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
}
