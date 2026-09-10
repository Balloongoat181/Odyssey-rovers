package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "Test drive", group = "Drive")
public class test_drive extends OpMode {

    private static final String BL_NAME = "backLeft";
    private static final String BR_NAME = "backRight";

    private DcMotorEx bl, br;

    @Override
    public void init() {

        // Drivetrain - only the two powered back wheels
        bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        br = hardwareMap.get(DcMotorEx.class, BR_NAME);

        // Zero power behavior
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor modes
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor directions
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Two-Wheel Drive Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ---------- Drivetrain (tank/differential style, no strafing) ----------
        // Forward/back comes from the left stick, turning comes from the right stick.
        // With only two powered wheels (and two unpowered omni wheels), strafing
        // isn't possible, so left_stick_x is intentionally unused.
        double y  = -gamepad1.left_stick_y;
        double rx =  gamepad1.right_stick_x;

        double blPower = y + rx;
        double brPower = y - rx;

        double max = Math.max(1.0, Math.max(Math.abs(blPower), Math.abs(brPower)));

        bl.setPower(blPower / max);
        br.setPower(brPower / max);

        telemetry.addData("Left Stick Y (drive)", y);
        telemetry.addData("Right Stick X (turn)", rx);
        telemetry.addData("Back Left Power", blPower / max);
        telemetry.addData("Back Right Power", brPower / max);
        telemetry.update();
    }
}
