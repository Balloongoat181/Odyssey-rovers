package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Mecanum Drive TeleOp", group = "Drive")
public class Bio extends OpMode {

    private static final String FL_NAME = "FL_NAME";
    private static final String FR_NAME = "FR_NAME";
    private static final String BL_NAME = "BL_NAME";
    private static final String BR_NAME = "BR_NAME";
    private static final String SHOOTER_NAME = "SHOOTER_NAME";
    private static final String FEEDER_NAME = "feeder";
    private static final String TAKE1_NAME = "TAKE1_NAME";
    private static final String TAKE2_NAME = "TAKE2_NAME";

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx shooter;

    private DcMotorEx intake, intake2;
    private CRServo feeder;

    // Shooter adjustable power
    private double shooterPower = 0.55;

    private double intakePower = 0.8;

    boolean shooterOn = false;

    boolean intakeOn = false;

    boolean intakeReversed = false;

    @Override
    public void init() {

        // Drivetrain
        fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
        bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        br = hardwareMap.get(DcMotorEx.class, BR_NAME);

        // Shooter
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        // Intake motors
        intake  = hardwareMap.get(DcMotorEx.class, TAKE1_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, TAKE2_NAME);

        // Gecko feed servos
        feeder  = hardwareMap.get(CRServo.class, FEEDER_NAME);

        // Zero power behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Motor modes
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset and enable shooter encoder
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor directions
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);

        // Servo directions — adjust if spinning wrong
        feeder.setDirection(CRServo.Direction.FORWARD);

        telemetry.addLine("Mecanum + Shooter + Feeder Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ---------- Drivetrain ----------
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double blPower = y - x + rx;
        double brPower = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(flPower),
                        Math.max(Math.abs(frPower),
                                Math.max(Math.abs(blPower), Math.abs(brPower)))));

        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);

        // ---------- Shooter power adjust ----------
        if (gamepad1.rightBumperWasPressed()) shooterPower += 0.05;
        if (gamepad1.leftBumperWasPressed()) shooterPower -= 0.05;

        shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));

        if (gamepad1.dpadUpWasPressed()) intakePower += .1;
        if (gamepad1.dpadDownWasPressed()) intakePower -= .1;

        intakePower = Math.max(0.0, Math.min(1.0, intakePower));

        // Toggle shooter
        if (gamepad1.bWasPressed()) {
            shooterOn = !shooterOn;
        }

        // Apply shooter state
        if (shooterOn) {
            shooter.setPower(shooterPower);
        } else {
            shooter.setPower(0);
        }

        if (gamepad1.aWasPressed()) {
            intakeOn = !intakeOn;
        }

        // Toggle intake direction with X button
        if (gamepad1.xWasPressed()) {
            intakeReversed = !intakeReversed;
        }

        // Apply intake power based on state and direction
        double effectiveIntakePower = intakeReversed ? -intakePower : intakePower;
        if (intakeOn) {
            intake.setPower(effectiveIntakePower);
            intake2.setPower(effectiveIntakePower);
        } else {
            intake.setPower(0);
            intake2.setPower(0);
        }

        // Feeder control (Triggers > 0.1)
        if (gamepad1.right_trigger > 0.1) {
            // Forward feed
            feeder.setPower(1.0);
        } else if (gamepad1.left_trigger > 0.1) {
            // Reverse feed
            feeder.setPower(-1.0);
        } else {
            // Stop feeding
            feeder.setPower(0);
        }

        // ---------- Telemetry ----------
        telemetry.addData("Shooter", "%s (Power: %.2f)", shooterOn ? "ON" : "OFF", shooterPower);
        telemetry.addData("Intake State", "%s [%s]",
                intakeOn ? "ON" : "OFF",
                intakeReversed ? "REVERSED" : "FORWARD");
        telemetry.addData("Intake Power", "%.2f", effectiveIntakePower);
        telemetry.addData("Feeder Power", "%.2f", feeder.getPower());
        telemetry.addData("Right Trigger", "%.2f (Pressed: %b)", gamepad1.right_trigger, gamepad1.right_trigger > 0.1);
        telemetry.addData("Left Trigger", "%.2f (Pressed: %b)", gamepad1.left_trigger, gamepad1.left_trigger > 0.1);
        telemetry.update();
    }
}