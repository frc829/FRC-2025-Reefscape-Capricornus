package digilib.intake;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;

public class IntakeConfig {

    public record Spark(@JsonProperty("name") String name, @JsonProperty("reduction") double reduction,
                        @JsonProperty("can-id") int canId,
                        @JsonProperty("smart-current-limit") int smartCurrentLimit,
                        @JsonProperty("brushless") boolean brushless,
                        @JsonProperty("brake-mode") boolean brakeMode,
                        @JsonProperty("inverted") boolean inverted, @JsonProperty("motor") String motor,
                        @JsonProperty("encoder-type") String encoderType,
                        @JsonProperty("motor-controller") String motorController,
                        @JsonProperty("encoder-depth") int depth,
                        @JsonProperty("encoder-period-ms") int periodMs,
                        @JsonProperty("control-period-seconds") double controlPeriodSeconds,
                        @JsonProperty("max-velocity-scalar") double maxVelocityScalar,
                        @JsonProperty("max-acceleration-scalar") double maxAccelerationScalar,
                        @JsonProperty("voltage-ks") double voltageKs,
                        @JsonProperty("voltage-kv") double voltageKv,
                        @JsonProperty("voltage-ka") double voltageKa,
                        @JsonProperty("velocity-voltage-kp") double velocityVoltageKp,
                        @JsonProperty("current-ks") double currentKs,
                        @JsonProperty("current-kv") double currentKv,
                        @JsonProperty("current-ka") double currentKa,
                        @JsonProperty("velocity-current-kp") double velocityCurrentKp) {

        public static Spark fromJsonFile(String jsonFileName) throws IOException {
            ObjectMapper mapper = new ObjectMapper();
            File file = new File(Filesystem.getDeployDirectory(), jsonFileName);
            return mapper.readValue(file, IntakeConfig.Spark.class);
        }

    }
}


