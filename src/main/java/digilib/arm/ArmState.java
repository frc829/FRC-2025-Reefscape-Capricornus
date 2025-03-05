package digilib.arm;

import com.ctre.phoenix6.signals.MagnetHealthValue;

public class ArmState {

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
    private double voltage = 0.0;
    private AbsoluteEncoderStatus absoluteEncoderStatus;

    public double getMotorEncoderPositionRotations() {
        return motorEncoderPositionRotations;
    }

    public double getMotorEncoderPositionDegrees() {
        return motorEncoderPositionRotations * 360.0;
    }

    public void setMotorEncoderPositionRotations(double motorEncoderPositionRotations) {
        this.motorEncoderPositionRotations = motorEncoderPositionRotations;
    }

    public double getMotorEncoderVelocityRPS() {
        return motorEncoderVelocityRPS;
    }

    public double getMotorEncoderVelocityDPS() {
        return motorEncoderVelocityRPS * 360.0;
    }

    public void setMotorEncoderVelocityRPS(double motorEncoderVelocityRPS) {
        this.motorEncoderVelocityRPS = motorEncoderVelocityRPS;
    }

    public double getAbsoluteEncoderPositionRotations() {
        return absoluteEncoderPositionRotations;
    }

    public double getAbsoluteEncoderPositionDegrees() {
        return absoluteEncoderPositionRotations * 360.0;
    }

    public void setAbsoluteEncoderPositionRotations(double absoluteEncoderPositionRotations) {
        this.absoluteEncoderPositionRotations = absoluteEncoderPositionRotations;
    }

    public double getAbsoluteEncoderVelocityRPS() {
        return absoluteEncoderVelocityRPS;
    }

    public double getAbsoluteEncoderVelocityDPS() {
        return absoluteEncoderVelocityRPS * 360.0;
    }

    public void setAbsoluteEncoderVelocityRPS(double absoluteEncoderVelocityRPS) {
        this.absoluteEncoderVelocityRPS = absoluteEncoderVelocityRPS;
    }

    public double getVoltage() {
        return voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public AbsoluteEncoderStatus getAbsoluteEncoderStatus() {
        return absoluteEncoderStatus;
    }

    public void setAbsoluteEncoderStatus(MagnetHealthValue magnetHealthValue) {
        this.absoluteEncoderStatus = AbsoluteEncoderStatus.fromMagnetHealthValue(magnetHealthValue);
    }
}

