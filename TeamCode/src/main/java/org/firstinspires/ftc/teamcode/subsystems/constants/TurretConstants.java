package org.firstinspires.ftc.teamcode.subsystems.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class TurretConstants {
    public static int SETPOINT_0 = 0;
    public static int SETPOINT_1 = 0;

    //TODO: tune pid

    public static double Kp = 0.006;
    public static double Ki = 0;
    public static double Kd = 0;

    public static int LOWER_LIMIT = -1000;
    public static int UPPER_LIMIT = 1000;
    public static double TICK_DEGREE_CONVERSION_FACTOR = 577.7/360;
}
