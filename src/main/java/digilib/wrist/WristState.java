package digilib.wrist;

import com.ctre.phoenix6.signals.MagnetHealthValue;

public class WristState {

    public enum AbsoluteEncoderStatus {
        GOOD,
        OK,
        BAD,
        REALLY_BAD;

        public static AbsoluteEncoderStatus fromMagnetHealthValue(MagnetHealthValue magnetHealthValue) {
            return switch (magnetHealthValue) {
                case Magnet_Green -> GOOD;
                case Magnet_Orange -> OK;
                case Magnet_Red -> BAD;
                default -> REALLY_BAD;
            };
        }
    }

    private double motorEncoderPositionRotations = 0.0;
    private double motorEncoderVelocityRPS = 0.0;
    private double absoluteEncoderPositionRotations = 0.0;
    private double absoluteEncoderVelocityRPS = 0.0;
    private double volts = 0.0;
    private double amps = 0.0;
    private AbsoluteEncoderStatus absoluteEncoderStatus;

    public double getMotorEncoderPositionRotations() {
        return motorEncoderPositionRotations;
    }

    public double getMotorEncoderPositionDegrees(){
        return motorEncoderPositionRotations * 360.0;
    }

    public void setMotorEncoderPositionRotations(double motorEncoderPositionRotations) {
        this.motorEncoderPositionRotations = motorEncoderPositionRotations;
    }

    public double getMotorEncoderVelocityRPS() {
        return motorEncoderVelocityRPS;
    }

    public double getMotorEncoderVelocityDPS(){
        return motorEncoderVelocityRPS * 360.0;
    }

    public void setMotorEncoderVelocityRPS(double motorEncoderVelocityRPS) {
        this.motorEncoderVelocityRPS = motorEncoderVelocityRPS;
    }

    public double getAbsoluteEncoderPositionRotations() {
        return absoluteEncoderPositionRotations;
    }

    public double getAbsoluteEncoderPositionDegrees(){
        return absoluteEncoderPositionRotations * 360.0;
    }

    public void setAbsoluteEncoderPositionRotations(double absoluteEncoderPositionRotations) {
        this.absoluteEncoderPositionRotations = absoluteEncoderPositionRotations;
    }

    public double getAbsoluteEncoderVelocityRPS() {
        return absoluteEncoderVelocityRPS;
    }

    public double getAbsoluteEncoderVelocityDPS(){
        return absoluteEncoderVelocityRPS * 360.0;
    }

    public void setAbsoluteEncoderVelocityRPS(double absoluteEncoderVelocityRPS) {
        this.absoluteEncoderVelocityRPS = absoluteEncoderVelocityRPS;
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

    public AbsoluteEncoderStatus getAbsoluteEncoderStatus() {
        return absoluteEncoderStatus;
    }

    public void setAbsoluteEncoderStatus(MagnetHealthValue magnetHealthValue) {
        this.absoluteEncoderStatus = AbsoluteEncoderStatus.fromMagnetHealthValue(magnetHealthValue);
    }
}
