package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Basic drive-only TeleOp
 *
 * gamepad1:
 *   Left stick      - drive / strafe
 *   Right stick X   - turn
 *   Left trigger    - slow mode (hold)
 */
@TeleOp(name = "M_Teleop Drive", group = "Drive")
public class M_TeleopDrive extends OpMode {

    private DcMotor fl, fr, bl, br;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        // Same directions as test_drive
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double scale = gamepad1.left_trigger > 0.1 ? 0.4 : 1.0;

        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double blPower = y - x + rx;
        double brPower = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))));

        fl.setPower(flPower / max * scale);
        fr.setPower(frPower / max * scale);
        bl.setPower(blPower / max * scale);
        br.setPower(brPower / max * scale);

        telemetry.addData("Slow Mode", scale < 1.0);
        telemetry.update();
    }
}
