package digilib.climber;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import digilib.MotorControllerType;
import digilib.elevator.SimulatedElevator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

import static digilib.MotorControllerType.*;

public class VortexClimber implements Climber {
    private final ClimberState state = new ClimberState();
    private final double minLengthMeters;
    private final double maxLengthMeters;
    private final double maxVelocityMPS;
    private final ClimberTelemetry telemetry;
    private final SparkFlex motor;
    private SparkFlexSim sparkFlexSim = null;
    private SimulatedElevator simClimber = null;

    public VortexClimber(ClimberConstants constants, SparkFlex motor) {
        this.minLengthMeters = constants.minLengthMeters();
        this.maxLengthMeters = constants.maxLengthMeters();
        this.maxVelocityMPS = constants.maxVelocityMPS();
        this.motor = motor;
        this.telemetry = new ClimberTelemetry(
                constants.name(),
                constants.minLengthMeters(),
                constants.maxLengthMeters(),
                constants.maxVelocityMPS(),
                constants.maxAccelerationMPSSquared());
        if (RobotBase.isSimulation()) {
            DCMotor dcMotor = DCMotor.getNeoVortex(1);
            sparkFlexSim = new SparkFlexSim(motor, dcMotor);
            sparkFlexSim.setPosition(constants.startingLengthMeters());
            simClimber = SimulatedElevator.createFromSysId(
                    0.0,
                    0.0,
                    constants.kvVoltsPerMPS(),
                    constants.kaVoltsPerMPSSquared(),
                    dcMotor,
                    constants.reduction(),
                    constants.startingLengthMeters(),
                    constants.minLengthMeters(),
                    constants.maxLengthMeters());
        }
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return REV_SPARK_FLEX;
    }

    @Override
    public double getMinLengthMeters() {
        return minLengthMeters;
    }

    @Override
    public double getMaxLengthMeters() {
        return maxLengthMeters;
    }

    @Override
    public double getMaxVelocityMPS() {
        return maxVelocityMPS;
    }

    @Override
    public ClimberState getState() {
        return state;
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.setMotorEncoderPositionMeters(motor.getEncoder().getPosition());
        state.setMotorEncoderVelocityMetersPerSecond(motor.getEncoder().getVelocity());
        state.setVolts(motor.getAppliedOutput() * motor.getBusVoltage());
        state.setAmps(motor.getOutputCurrent());
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {
        var inputVoltage = motor.getAppliedOutput() * 12.0;
        simClimber.setInputVoltage(inputVoltage);
        simClimber.update(dt);
        sparkFlexSim.iterate(simClimber.getVelocityMetersPerSecond(), 12.0, dt);
    }
}
