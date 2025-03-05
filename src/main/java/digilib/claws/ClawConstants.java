package digilib.claws;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static digilib.claws.ClawState.*;

public record ClawConstants(String name,
                            PneumaticsModuleType moduleType,
                            ClawValue solenoidOnClawValue) {
}
