package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL_NAME");
        c.frontRightName.set("FR_NAME");
        c.backLeftName.set("BL_NAME");
        c.backRightName.set("BR_NAME");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.8693507937934457);
        c.yPodOffset.set(4.134970837690699);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(() -> TunableConstants.forwardP);
                Controller secondaryTranslationalForward = Controller.proportional(() -> TunableConstants.forwardSecondaryP);
                Controller primaryTranslationalLateral = Controller.proportional(() -> TunableConstants.strafeP);
                Controller secondaryTranslationalLateral = Controller.proportional(() -> TunableConstants.strafeSecondaryP);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(() -> TunableConstants.coastFF));
                c.brake.set(Controller.proportionalFeedforward(() -> TunableConstants.brakeFF));

                c.headingFeedback.set(Controller.proportional(() -> TunableConstants.headingP));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0733623833422629, -0.001998221650928741));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0728868348804133, 0.0480624999735906));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016561558250785735, 0.0019780226081605795));

                c.maxAchievableForwardVelocity.set(58.86863284485901);
                c.maxAchievableStrafeVelocity.set(50.1318885447436);
                c.naturalForwardDeceleration.set(59.401310709080086);
                c.naturalStrafeDeceleration.set(64.82929492731607);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}