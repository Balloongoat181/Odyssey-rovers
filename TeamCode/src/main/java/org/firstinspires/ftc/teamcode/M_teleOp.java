package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "Mecanum Drive TeleOp", group = "Drive")
public class M_teleOp extends OpMode {

    private static final String FL_NAME = "frontLeft";
    private static final String FR_NAME = "frontRight";
    private static final String BL_NAME = "backLeft";
    private static final String BR_NAME = "backRight";

    private static final String SHOOTER_NAME = "shooter";

    // Gecko feed servos
    private static final String FEEDER_NAME  = "Feeder";


    // Light indicator
    private static final String LIGHT_NAME = "shooterLight";

    private static final String INTAKE_NAME = "Intake";

    private static final String GATE_NAME = "Gate";

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx shooter;
    private DcMotorEx intake;

    private DcMotorEx feeder;

    private CRServo gate;

    private Servo shooterLight;

    // Pedro Pathing variables
    private Follower follower;
    private boolean positionLocked = false;
    private Pose targetPose;


    // Shooter adjustable power
    private double shooterPower = 0.6;

    // Maximum velocity your shooter can reach at full power (adjust based on testing)
    private static final double MAX_VELOCITY = 2360.0; // ticks per second at 100% power
    private static final double VELOCITY_TOLERANCE = 100.0; // tolerance range

    private long shooterAtSpeedTime = 0;
    private boolean wasAtSpeed = false;
    private static final long SPEED_STABLE_DURATION = 2000; // milliseconds

    // Bumper edge detection
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    boolean shooterOn = false;
    boolean intakeOn = false;
    boolean feederOn = false;

    @Override
    public void init() {

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);



        // Drivetrain
        fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
        bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        br = hardwareMap.get(DcMotorEx.class, BR_NAME);

        // Shooter
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        gate = hardwareMap.get(CRServo.class, GATE_NAME);

        // Feeder
        feeder = hardwareMap.get(DcMotorEx.class, FEEDER_NAME);


        // RGB Indicator Light
        shooterLight = hardwareMap.get(Servo.class, LIGHT_NAME);

        // Zero power behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Motor modes
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset and enable shooter encoder
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor directions
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        gate.setDirection(DcMotorSimple.Direction.REVERSE);


        // Feeder direction
        feeder.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Mecanum + Shooter + Feeder + Intake Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Update follower localization
        follower.update();



        // ---------- Position Lock Toggle (A Button) ----------
        if (gamepad1.aWasPressed()) {
            positionLocked = !positionLocked;

            if (positionLocked) {
                targetPose = follower.getPose();
                follower.holdPoint(targetPose);
            } else {
                follower.breakFollowing();
            }
        }


        // ---------- Drivetrain ----------
        if (positionLocked) {
            // Position hold mode - Pedro controls the robot
        } else {
            // Normal driver control
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
        }

        // ---------- Shooter power adjust ----------
        if (gamepad1.right_bumper && !lastRightBumper) shooterPower += 0.1;
        if (gamepad1.left_bumper && !lastLeftBumper) shooterPower -= 0.1;

        shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

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

        // ---------- Shooter Speed Light Control ----------
        double targetVelocity = shooterPower * MAX_VELOCITY;
        double currentVelocity = shooter.getVelocity();
        boolean atSpeed = shooterOn &&
                Math.abs(currentVelocity) >= targetVelocity - VELOCITY_TOLERANCE;

        // Track how long we've been at speed
        if (atSpeed && !wasAtSpeed) {
            shooterAtSpeedTime = System.currentTimeMillis();
        }

        if (!atSpeed) {
            shooterAtSpeedTime = 0;
        }

        wasAtSpeed = atSpeed;

        // Set light color based on state
        if (atSpeed && (System.currentTimeMillis() - shooterAtSpeedTime >= SPEED_STABLE_DURATION)) {
            shooterLight.setPosition(0.42);  // green - stable at speed for 2 seconds
        } else if (shooterOn) {
            shooterLight.setPosition(0.30);  // red - shooter on but not at speed yet
        } else {
            shooterLight.setPosition(0.60);  // blue - shooter off
        }

        // ---------- Intake Control ----------
        if (gamepad1.yWasPressed()) {
            intakeOn = !intakeOn;
        }

        // Apply intake state
        if (intakeOn) {
            intake.setPower(0.6);
        } else {
            intake.setPower(0);
        }

        // ---------- Feeder Control ----------
        if (gamepad1.xWasPressed()) {
            feederOn = !feederOn;
        }

        // Apply feeder state (only feed when shooter is on)
        if (feederOn && shooterOn) {
            feeder.setPower(shooterPower);
        } else {
            feeder.setPower(0);
        }

        // ---------- Telemetry ----------
        // Position info


        // Navigation state
        // Shoot pose saved message (show for 2 seconds)


        // Shoot pose info


        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("At Speed", atSpeed);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Feeder On", feederOn);
        telemetry.update();
    }
}