package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "M_Telop", group = "Drive")
public class test_drive extends OpMode {

    // Hardware Names
    private static final String BL_NAME = "backLeft";
    private static final String BR_NAME = "backRight";
    private static final String FL_NAME = "frontLeft";
    private static final String FR_NAME = "frontRight";
    private static final String INTAKE_NAME = "intake";
    private static final String INTAKE2_NAME = "intake2";
    private static final String SHOOTER_NAME = "shooter";
    private static final String FEEDER_NAME = "feeder";

    // Hardware Objects
    private DcMotorEx bl, br, fl, fr;
    private DcMotorEx intake, intake2;
    private DcMotorEx shooter;
    private CRServo feeder;

    // State Variables
    private double intakePower = 0.5;
    private double shooterPower = 0.8;
    private boolean intakeOn = false;
    private boolean shooterOn = false;
    private boolean flip = false;


    @Override
    public void init() {
        // Drivetrain Initialization
        bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        br = hardwareMap.get(DcMotorEx.class, BR_NAME);
        fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        fr = hardwareMap.get(DcMotorEx.class, FR_NAME);

        // Mechanisms Initialization
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake2 = hardwareMap.get(DcMotorEx.class, INTAKE2_NAME);
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        feeder = hardwareMap.get(CRServo.class, FEEDER_NAME);

        // Drive Motor Directions (Typical for Mecanum)
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        // Zero Power Behavior
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Motor Modes
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servo Directions (Assumed feeding inward)
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Robot Initialized - bioBuzz Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ------------------ DRIVETRAIN (Mecanum) ------------------
        double y = -gamepad1.left_stick_y; // Forward/Back
        double x = gamepad1.left_stick_x;  // Strafe
        double rx = gamepad1.right_stick_x; // Turn

        // Slow Mode Modifier: Hold Left Trigger to slow down for precision
        double driveScale = 0.8 - (gamepad1.left_trigger * 0.5); 

        double flPower = (y + x + rx) * driveScale;
        double frPower = (y - x - rx) * driveScale;
        double blPower = (y - x + rx) * driveScale;
        double brPower = (y + x - rx) * driveScale;

        // Normalize powers so they don't exceed 1.0
        double max = Math.max(1.0, Math.max(Math.abs(flPower), 
                     Math.max(Math.abs(frPower), 
                     Math.max(Math.abs(blPower), Math.abs(brPower)))));

        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);


        // ------------------ INTAKE CONTROL ------------------
        // Toggle Intake: Button B
        if (gamepad1.bWasPressed()) {
            intakeOn = !intakeOn;
        }
        // Flip Direction: Button A
        if (gamepad1.aWasPressed()) {
            flip = !flip;
        }
        // Adjust Power: Bumpers
        if (gamepad1.rightBumperWasPressed()) {
            intakePower += 0.1;
        }
        if (gamepad1.leftBumperWasPressed()) {
            intakePower -= 0.1;
        }
        intakePower = Math.max(0, Math.min(1.0, intakePower));

        double appliedIntakePower = flip ? -intakePower : intakePower;
        intake.setPower(intakeOn ? appliedIntakePower : 0);
        intake2.setPower(intakeOn ? appliedIntakePower : 0);


        // ------------------ SHOOTER (Flywheel) ------------------
        // Toggle Shooter: Button X
        if (gamepad1.xWasPressed()) {
            shooterOn = !shooterOn;
        }
        // Adjust Shooter Power: D-pad Up/Down
        if (gamepad1.dpadUpWasPressed()) {
            shooterPower += 0.05;
        }
        if (gamepad1.dpadDownWasPressed()) {
            shooterPower -= 0.05;
        }
        shooterPower = Math.max(0, Math.min(1.0, shooterPower));
        
        shooter.setPower(shooterOn ? shooterPower : 0);


        // ------------------ FEED SERVOS ------------------
        // Run Feed Servos while Right Trigger is held
        if (gamepad1.right_trigger > 0.1) {
            feeder.setPower(1);
        } else {
            feeder.setPower(0);
        }

        // ------------------ TELEMETRY ------------------
        telemetry.addData("Drive Scale", "%.2f", driveScale);
        telemetry.addData("Intake", "%s (Power: %.2f)", intakeOn ? "ON" : "OFF", intakePower);
        telemetry.addData("Flip", flip ? "ON" : "OFF");
        telemetry.addData("Shooter", "%s (Power: %.2f)", shooterOn ? "ON" : "OFF", shooterPower);
        telemetry.addData("Feeder", "%s", (gamepad1.right_trigger > 0.1) ? "RUNNING" : "STOPPED");
        telemetry.update();
    }
}
