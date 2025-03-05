package digilib.elevator;

public class ElevatorState {

    private double motorEncoderPositionMeters = 0.0;
    private double motorEncoderVelocityMPS = 0.0;
    private double voltage = 0.0;

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

    public double getVoltage() {
        return voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
