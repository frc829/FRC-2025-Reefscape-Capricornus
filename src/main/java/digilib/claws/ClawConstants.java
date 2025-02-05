package digilib.claws;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import digilib.claws.ClawState.ClawValue;

import java.util.HashMap;
import java.util.Map;

public class ClawConstants {

    private final String name;
    private final int module;
    private final PneumaticsModuleType moduleType;
    private final Map<Boolean, ClawValue> solenoidClawValueMap;
    private final Map<ClawValue, Boolean> clawValueSolenoidMap;
    private final ClawValue startingValue;

    public ClawConstants(
            String name,
            int module,
            PneumaticsModuleType moduleType,
            Map<ClawValue, Boolean> clawValueSolenoidMap,
            ClawValue startingState) {
        this.name = name;
        this.module = module;
        this.moduleType = moduleType;
        this.clawValueSolenoidMap = clawValueSolenoidMap;
        solenoidClawValueMap = new HashMap<Boolean, ClawValue>();
        solenoidClawValueMap.put(clawValueSolenoidMap.get(ClawValue.OPEN), ClawValue.OPEN);
        solenoidClawValueMap.put(clawValueSolenoidMap.get(ClawValue.CLOSED), ClawValue.CLOSED);
        this.startingValue = startingState;
    }

    public Map<ClawValue, Boolean> getClawValueSolenoidMap() {
        return clawValueSolenoidMap;
    }

    public Map<Boolean, ClawValue> getSolenoidClawValueMap() {
        return solenoidClawValueMap;
    }

    public ClawValue getStartingValue() {
        return startingValue;
    }

    public String getName() {
        return name;
    }

    public int getModule() {
        return module;
    }

    public PneumaticsModuleType getModuleType() {
        return moduleType;
    }


}
