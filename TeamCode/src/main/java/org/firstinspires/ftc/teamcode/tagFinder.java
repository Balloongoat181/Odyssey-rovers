package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.AbstractBijectiveMap;
import com.pedropathing.math.MathFunctions; // used for clamp()
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "tag finder", group = "Drive")
public class tagFinder extends OpMode {

    private static final String SHOOTER_NAME = "shooter";
    private static final String SHOOTER2_NAME= "shooter2";

    // Gecko feed servos
    private static final String FEED_LEFT_NAME = "feedLeft";
    private static final String FEED_RIGHT_NAME = "feedRight";

    // Light indicator
    private static final String LIGHT_NAME = "shooterLight";

    private DcMotorEx shooter;

    private DcMotorEx shooter2;

    private Limelight3A limelight;
    private Follower follower;

    private CRServo feedLeft, feedRight;
    private Servo shooterLight;

    // Goal positions (field coordinates, inches)
    private static final Pose BLUE_GOAL = new Pose(15.0, 131.0);
    private static final Pose RED_GOAL  = new Pose(129.0, 131.0);

    // Distance (inches) → shooter velocity (ticks/sec) interpolation map.
    // Adjust these values based on testing your specific shooter.
    private AbstractBijectiveMap.NumericBijectiveMap distVelMap;

    // Manual fine-tune offset applied on top of interpolated velocity (bumpers)
    private double velocityTrimOffset = 0.0;
    private static final double TRIM_STEP = 100.0; // ticks/sec per bumper press

    // Maximum velocity your shooter can reach (used for shooter2 power scaling)
    private static final double MAX_VELOCITY = 2360.0; // ticks per second at 100% power
    private static final double VELOCITY_TOLERANCE = 100.0; // tolerance range

    private long shooterAtSpeedTime = 0;
    private boolean wasAtSpeed = false;
    private static final long SPEED_STABLE_DURATION = 2000; // milliseconds

    // Bumper edge detection
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    boolean shooterOn = false;

    // Tag alignment control
    private boolean alignmentActive = false;
    private double targetHeading = 0;

    // Alliance selection — toggles with gamepad1.start
    // true = Red (tag 24), false = Blue (tag 20)
    private boolean isRedAlliance = true;
    private static final int RED_TAG_ID = 24;
    private static final int BLUE_TAG_ID = 20;

    // Deadband: don't rotate if the tag is within this many degrees of center
    private static final double ALIGNMENT_DEADBAND_DEG = 0.7;

    @Override
    public void init() {

        // Create Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        // Shooter
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, SHOOTER2_NAME);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Velocity PIDF — F = 32767 / MAX_VELOCITY, tune P if response is sluggish
        shooter.setVelocityPIDFCoefficients(20, 0, 0, 32767.0 / MAX_VELOCITY);

        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        shooterLight = hardwareMap.get(Servo.class, LIGHT_NAME);

        // Gecko feed servos
        feedLeft = hardwareMap.get(CRServo.class, FEED_LEFT_NAME);
        feedRight = hardwareMap.get(CRServo.class, FEED_RIGHT_NAME);

        // Servo directions — adjust if spinning wrong
        feedLeft.setDirection(CRServo.Direction.FORWARD);
        feedRight.setDirection(CRServo.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Populate distance → velocity map (distance in inches, velocity in ticks/sec)
        distVelMap = new AbstractBijectiveMap.NumericBijectiveMap();
        distVelMap.put(20.0, 1200.0);
        distVelMap.put(40.0, 1700.0);
        distVelMap.put(60.0, 2100.0);
        distVelMap.put(80.0, 2360.0);

        telemetry.addLine("Pedro Pathing + Shooter + Feeder Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        // Required by Pedro Pathing before you can call setTeleOpDrive in loop()
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // ---------- Drivetrain with Pedro Pathing ----------
        double y = -gamepad1.left_stick_y;   // Forward/backward
        double x = -gamepad1.left_stick_x;   // Strafe left/right
        double rx = -gamepad1.right_stick_x;  // Rotation (used when alignment OFF)

        // ---------- Alliance Toggle (Start Button) ----------
        if (gamepad1.startWasPressed()) {
            isRedAlliance = !isRedAlliance;
        }
        int targetTagId = isRedAlliance ? RED_TAG_ID : BLUE_TAG_ID;

        double currentHeading = follower.getPose().getHeading();

        // ---------- Tag Alignment Toggle (A Button) ----------
        if (gamepad1.aWasPressed()) {
            alignmentActive = !alignmentActive;
            if (alignmentActive) {
                targetHeading = currentHeading;
            } else {
                follower.breakFollowing();
                follower.startTeleopDrive();
            }
        }

        if (alignmentActive) {
            LLResult result = limelight.getLatestResult();
            boolean tagFound = false;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetTagId) {
                        double tx = fiducial.getTargetXDegrees();

                        if (Math.abs(tx) > ALIGNMENT_DEADBAND_DEG) {
                            // If the robot spins the wrong way, change minus to plus
                            targetHeading = currentHeading - Math.toRadians(tx);
                        }
                        tagFound = true;
                        break;
                    }
                }
            }

            // Compute heading error, normalize to [-π, π]
            double headingError = targetHeading - currentHeading;
            while (headingError >  Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            double rotationOutput = Math.max(-1.0, Math.min(1.0,
                    Constants.followerConstants.getCoefficientsHeadingPIDF().P * headingError));

            if (tagFound) {
                // Tag visible — PID correction locks heading, right stick ignored
                follower.setTeleOpDrive(y, x, rotationOutput, true);
            } else if (Math.abs(rx) > 0.05) {
                // Tag not visible, driver rotating to search — allow it
                follower.setTeleOpDrive(y, x, rx, true);
                targetHeading = currentHeading;
            } else {
                // Tag not visible, no manual input — hold last known target heading
                follower.setTeleOpDrive(y, x, rotationOutput, true);
            }
        } else {
            // Alignment OFF — full manual control
            follower.setTeleOpDrive(y, x, rx, true);
            targetHeading = currentHeading;
        }

        follower.update();

        // ---------- Shooter velocity trim (bumpers fine-tune offset) ----------
        if (gamepad1.right_bumper && !lastRightBumper) velocityTrimOffset += TRIM_STEP;
        if (gamepad1.left_bumper && !lastLeftBumper)  velocityTrimOffset -= TRIM_STEP;

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper  = gamepad1.left_bumper;

        // ---------- Distance-based target velocity ----------
        Pose currentPose = follower.getPose();
        Pose goal = isRedAlliance ? RED_GOAL : BLUE_GOAL;
        double distanceToGoal = Math.hypot(currentPose.getX() - goal.getX(), currentPose.getY() - goal.getY());

        double targetVelocity = MathFunctions.clamp(
                getVelocityForDistance(distanceToGoal) + velocityTrimOffset,
                0, MAX_VELOCITY);

        // ---------- Shooter Speed Light Control ----------
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

        // Toggle shooter (B button)
        if (gamepad1.bWasPressed()) {
            shooterOn = !shooterOn;
        }

        // Apply shooter state
        if (shooterOn) {
            shooter.setVelocity(targetVelocity);
            shooter2.setPower(targetVelocity / MAX_VELOCITY); // no encoder, scale to 0-1
        } else {
            shooter.setVelocity(0);
            shooter2.setPower(0);
        }

        // ---------- Gecko Feed Servos ----------
        if (gamepad1.y) {
            // Forward feed
            feedLeft.setPower(1.0);
            feedRight.setPower(-1.0);
        } else if (gamepad1.x) {
            // Reverse feed
            feedLeft.setPower(-1.0);
            feedRight.setPower(1.0);
        } else {
            // Stop feeding
            feedLeft.setPower(0);
            feedRight.setPower(0);
        }

        // ---------- Debug Telemetry ----------
        telemetry.addData("Y pressed", gamepad1.y);
        telemetry.addData("X pressed", gamepad1.x);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Distance to Goal (in)", String.format("%.1f", distanceToGoal));
        telemetry.addData("Target Velocity (t/s)", String.format("%.0f", targetVelocity));
        telemetry.addData("Current Velocity (t/s)", String.format("%.0f", currentVelocity));
        telemetry.addData("Velocity Trim Offset", velocityTrimOffset);
        telemetry.addData("Feeder L Power", feedLeft.getPower());
        telemetry.addData("Feeder R Power", feedRight.getPower());

        // Pedro telemetry
        telemetry.addData("Current Pose", follower.getPose());
        telemetry.addData("Current Heading (deg)", Math.toDegrees(currentHeading));
        telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Target Tag ID", targetTagId);
        telemetry.addData("Alignment Active", alignmentActive);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // Find and display data for the target tag only
            boolean foundTarget = false;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == targetTagId) {
                    telemetry.addData("Tag ID", fiducial.getFiducialId());
                    telemetry.addData("TX (deg)", fiducial.getTargetXDegrees());
                    telemetry.addData("TY (deg)", fiducial.getTargetYDegrees());
                    telemetry.addData("TA (%)", fiducial.getTargetArea());
                    foundTarget = true;
                    break;
                }
            }
            if (!foundTarget) {
                telemetry.addData("Target Tag", "Not in view");
            }
        } else {
            telemetry.addData("Limelight", "No valid result");
        }

        telemetry.update();
    }

    /**
     * Interpolates shooter velocity from the distVelMap using Pedro Pathing's
     * NumericBijectiveMap.interpolateKey(). Automatically clamps to map bounds
     * when distance is outside the defined range.
     */
    private double getVelocityForDistance(double distance) {
        return distVelMap.interpolateKey(distance);
    }
}