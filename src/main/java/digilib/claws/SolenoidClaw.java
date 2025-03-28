package digilib.claws;

import edu.wpi.first.wpilibj.Solenoid;

public class SolenoidClaw extends Claw {
    private final Solenoid solenoid;

    public SolenoidClaw(
            Config config,
            Solenoid solenoid) {
        super(config.name(), config.solenoidOnValue());
        this.solenoid = solenoid;
    }

    @Override
    public void setValue(Value value) {
        solenoid.set(value == valueWhenSolenoidOn);
    }

    @Override
    public void toggle() {
        solenoid.toggle();
    }

    @Override
    public void update() {
        value = getValue();
        super.update();
    }
}
