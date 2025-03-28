package digilib.wrist;

public class WristState {

    private double motorEncoderPositionRotations = 0.0;
    private double motorEncoderVelocityRPS = 0.0;
    private double volts = 0.0;
    private double amps = 0.0;

    public double getMotorEncoderPositionRotations() {
        return motorEncoderPositionRotations;
    }

    public double getMotorEncoderPositionDegrees(){
        return motorEncoderPositionRotations * 360.0;
    }

    public void setMotorEncoderPositionRotations(double motorEncoderPositionRotations) {
        this.motorEncoderPositionRotations = motorEncoderPositionRotations;
    }

    public double getMotorEncoderVelocityDPS(){
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
