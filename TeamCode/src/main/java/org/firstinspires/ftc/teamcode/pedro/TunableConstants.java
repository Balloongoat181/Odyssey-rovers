package org.firstinspires.ftc.teamcode.pedro;

import com.acmerobotics.dashboard.config.Config;

/**
 * Live-tunable Foresight gains, editable from the FTC Dashboard "Configuration" panel
 * while the robot is running (no redeploy needed). Values here are the defaults produced
 * by the Pedro autotune procedures in {@link Tuning} - after running a tuner, either paste
 * its result here, or nudge the fields live from the dashboard and copy the final numbers
 * back into this file so they persist across restarts.
 */
@Config
public class TunableConstants {
    public static double forwardP = 0.33835067485637227;
    public static double forwardSecondaryP = 0.1250115334628888;

    public static double strafeP = 0.5221910149361718;
    public static double strafeSecondaryP = 0.1929356268180166;

    public static double headingP = 6.52844832805518;

    public static double coastFF = 0.017820612627623536;
    public static double brakeFF = 0.015147520733480005;
}