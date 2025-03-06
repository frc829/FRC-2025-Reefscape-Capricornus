package digilib.climber;

public class ClimberState {

    private double motorEncoderPositionMeters = 0.0;
    private double motorEncoderVelocityMetersPerSecond = 0.0;
    private double volts = 0.0;
    private double amps = 0.0;

    public double getMotorEncoderPositionMeters() {
        return motorEncoderPositionMeters;
    }

    public void setMotorEncoderPositionMeters(double motorEncoderPositionMeters) {
        this.motorEncoderPositionMeters = motorEncoderPositionMeters;
    }

    public double getMotorEncoderVelocityMetersPerSecond() {
        return motorEncoderVelocityMetersPerSecond;
    }

    public void setMotorEncoderVelocityMetersPerSecond(double motorEncoderVelocityMetersPerSecond) {
        this.motorEncoderVelocityMetersPerSecond = motorEncoderVelocityMetersPerSecond;
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
