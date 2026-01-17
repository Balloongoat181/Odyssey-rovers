package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Mecanum Drive TeleOp", group = "Drive")
public class MecanumDriveTeleOp_twodrive extends OpMode {

    private static final String FL_NAME = "frontLeft";
    private static final String FR_NAME = "frontRight";
    private static final String BL_NAME = "backLeft";
    private static final String BR_NAME = "backRight";

    private static final String SHOOTER_NAME = "shooter";

    // Gecko feed servos
    private static final String FEED_LEFT_NAME  = "feedLeft";
    private static final String FEED_RIGHT_NAME = "feedRight";

    // Light indicator
    private static final String LIGHT_NAME = "shooterLight";

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx shooter;

    private CRServo feedLeft, feedRight;
    private Servo shooterLight;

    // Pedro Pathing variables
    private Follower follower;
    private boolean positionLocked = false;
    private boolean aButtonPrevious = false;
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

    @Override
    public void init() {

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Drivetrain
        fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
        bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        br = hardwareMap.get(DcMotorEx.class, BR_NAME);

        // Shooter
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        // Gecko feed servos
        feedLeft  = hardwareMap.get(CRServo.class, FEED_LEFT_NAME);
        feedRight = hardwareMap.get(CRServo.class, FEED_RIGHT_NAME);

        // RGB Indicator Light
        shooterLight = hardwareMap.get(Servo.class, LIGHT_NAME);

        // Zero power behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Motor modes
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset and enable shooter encoder
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Motor directions
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.FORWARD);

        // Servo directions — adjust if spinning wrong
        feedLeft.setDirection(CRServo.Direction.FORWARD);
        feedRight.setDirection(CRServo.Direction.FORWARD);

        telemetry.addLine("Mecanum + Shooter + Feeder + Pedro Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Update follower localization
        follower.update();

        // ---------- Position Lock Toggle (A Button) ----------
        if (gamepad2.a && !aButtonPrevious) {
            positionLocked = !positionLocked;

            if (positionLocked) {
                // Capture CURRENT position when pressing A
                targetPose = follower.getPose();
                follower.holdPoint(targetPose);
            } else {
                // Unlocking - tell Pedro to stop holding
                follower.breakFollowing();
            }
        }
        aButtonPrevious = gamepad2.a;

        // ---------- Drivetrain ----------
        if (positionLocked) {
            // Position hold mode - Pedro controls the robot
            // follower.update() above handles holding the position

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
        if (gamepad2.right_bumper && !lastRightBumper) shooterPower += 0.05;
        if (gamepad2.left_bumper && !lastLeftBumper) shooterPower -= 0.05;

        shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));

        lastRightBumper = gamepad2.right_bumper;
        lastLeftBumper = gamepad2.left_bumper;

        // Toggle shooter
        if (gamepad2.bWasPressed()) {
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
            // Just reached speed - start timer
            shooterAtSpeedTime = System.currentTimeMillis();
        }

        if (!atSpeed) {
            // Reset timer when not at speed
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

        // ---------- Gecko Feed Servos ----------
        if (gamepad2.y) {
            // Forward feed
            feedLeft.setPower(1.0);
            feedRight.setPower(-1.0);
        }
        else if (gamepad2.x) {
            // Reverse feed (Xbox/Logitech X button)
            feedLeft.setPower(-1.0);
            feedRight.setPower(1.0);
        }
        else {
            // Stop feeding
            feedLeft.setPower(0);
            feedRight.setPower(0);
        }

        // ---------- Debug Telemetry ----------
        telemetry.addData("Position Locked", positionLocked);
        telemetry.addData("Y pressed", gamepad1.y);
        telemetry.addData("X pressed", gamepad1.x);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("At Speed", atSpeed);
        telemetry.addData("Feeder L Power", feedLeft.getPower());
        telemetry.addData("Feeder R Power", feedRight.getPower());
        telemetry.update();
    }
}

