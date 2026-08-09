package frc.robot.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Retries a Phoenix6 config-apply call a few times before giving up, instead of silently
 * accepting whatever the first attempt returns. A dropped config write (e.g. from CAN bus
 * traffic at boot) otherwise leaves a motor controller running with default settings --
 * unset soft limits, unset current limits, unset PID gains -- with nothing to show for it
 * except a return value nobody checked.
 */
public final class PhoenixUtil {
    private static final int kMaxAttempts = 5;

    private PhoenixUtil() {}

    /**
     * @param deviceDescription Human-readable label used in the Driver Station error if every
     *     attempt fails (e.g. "Shooter lead (CAN 27)").
     * @param command The config-apply call to retry, e.g. {@code () -> motor.getConfigurator().apply(config)}.
     */
    public static void tryUntilOk(String deviceDescription, Supplier<StatusCode> command) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int attempt = 0; attempt < kMaxAttempts && !status.isOK(); attempt++) {
            status = command.get();
        }
        if (!status.isOK()) {
            DriverStation.reportError(
                deviceDescription + " config failed after " + kMaxAttempts + " attempts: " + status,
                false
            );
        }
    }
}
