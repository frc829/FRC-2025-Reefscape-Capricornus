package digilib.winch;

import com.revrobotics.spark.SparkFlex;
import digilib.MotorControllerType;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.Timer;

import static digilib.MotorControllerType.*;

public class VortexWinch implements Winch {
    private final WinchState state = new WinchState();
    private final SparkFlex sparkFlex;
    private final WinchTelemetry telemetry;
    private WinchRequest request;

    public VortexWinch(WinchConstants constants, SparkFlex sparkFlex) {
        this.sparkFlex = sparkFlex;
        this.telemetry = new WinchTelemetry(constants.name());
    }

    @Override
    public MotorControllerType getMotorControllerType() {
        return REV_SPARK_FLEX;
    }

    @Override
    public WinchState getState() {
        return state;
    }

    @Override
    public void setControl(WinchRequest request) {
        if (this.request != request) {
            this.request = request;
        }
        request.apply(this);
    }

    @Override
    public void setDutyCycle(Dimensionless dutyCycle) {
        sparkFlex.set(dutyCycle.baseUnitMagnitude());
    }

    @Override
    public void setIdle() {
        sparkFlex.set(0);
    }

    @Override
    public void update() {
        updateState();
        updateTelemetry();
    }

    @Override
    public void updateState() {
        state.withDutyCycle(sparkFlex.get())
                .withVoltage(sparkFlex.getAppliedOutput() * sparkFlex.getBusVoltage())
                .withTimestamp(Timer.getFPGATimestamp());
    }

    @Override
    public void updateTelemetry() {
        telemetry.telemeterize(state);
    }

    @Override
    public void updateSimState(double dt, double supplyVoltage) {

    }
}
