package digilib.claws;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public record ClawConstants(String name,
                            PneumaticsModuleType moduleType,
                            ClawValue solenoidOnClawValue) {
}
