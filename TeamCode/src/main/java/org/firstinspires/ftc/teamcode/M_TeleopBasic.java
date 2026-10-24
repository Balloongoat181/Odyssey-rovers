package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Basic TeleOp
 *
 * gamepad1:
 *   Left stick      - drive / strafe
 *   Right stick X   - turn
 *   Left trigger    - slow mode (hold)
 *   Right bumper    - intake in (hold)
 *   Left bumper     - intake out (hold)
 *   X               - toggle shooter
 *   Right trigger   - run feeder (hold)
 */
@TeleOp(name = "M_Teleop Basic", group = "Drive")
public class M_TeleopBasic extends OpMode {

    private DcMotor fl, fr, bl, br;
    private DcMotor intake, intake2, shooter;
    private CRServo feeder;

    private boolean shooterOn = false;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        feeder = hardwareMap.get(CRServo.class, "feeder");

        // Same directions as test_drive
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive (mecanum)
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

        // Intake
        double intakePower = 0;
        if (gamepad1.right_bumper) {
            intakePower = 1.0;
        } else if (gamepad1.left_bumper) {
            intakePower = -1.0;
        }
        intake.setPower(intakePower);
        intake2.setPower(intakePower);

        // Shooter
        if (gamepad1.xWasPressed()) {
            shooterOn = !shooterOn;
        }
        shooter.setPower(shooterOn ? 0.8 : 0);

        // Feeder
        feeder.setPower(gamepad1.right_trigger > 0.1 ? 1.0 : 0);

        telemetry.addData("Slow Mode", scale < 1.0);
        telemetry.addData("Intake", intakePower);
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.update();
    }
}
