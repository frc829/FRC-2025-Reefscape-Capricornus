package digilib.climber;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import digilib.elevator.SimulatedElevator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

public class VortexClimber extends Climber {
    private final SparkFlex motor;
    private SparkFlexSim sparkFlexSim = null;
    private SimulatedElevator simClimber = null;

    public VortexClimber(Config config, SparkFlex motor) {
        super(config.name(),
                config.minLengthMeters(),
                config.maxLengthMeters(),
                config.maxVelocityMPS(),
                config.maxAccelerationMPSSquared());
        this.motor = motor;
        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeoVortex(1);
            sparkFlexSim = new SparkFlexSim(motor, dcMotor);
            sparkFlexSim.setPosition(config.startingLengthMeters());
            simClimber = SimulatedElevator.createFromSysId(
                    0.0,
                    0.0,
                    config.kvVoltsPerMPS(),
                    config.kaVoltsPerMPSSquared(),
                    dcMotor,
                    config.startingLengthMeters(),
                    config.minLengthMeters(),
                    config.maxLengthMeters());
        }
    }

    @Override
    public double getMotorEncoderPositionMeters() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getMotorEncoderVelocityMetersPerSecond() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public double getVolts() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public void applyVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public double getAmps() {
        return motor.getOutputCurrent();
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getAppliedOutput() * supplyVoltage;
        simClimber.setInputVoltage(inputVoltage);
        simClimber.update(dt);
        sparkFlexSim.iterate(simClimber.getVelocityMetersPerSecond(), supplyVoltage, dt);
    }
}
