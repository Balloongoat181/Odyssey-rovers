package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
@TeleOp(name = "tag finder", group = "Drive")
public class tagFinder extends OpMode {

    // ========== NESTED CONFIG GROUPS ==========
    @Configurable
    public static class ShooterPID {
        public static double P = 23;
        public static double I = 0.0;
        public static double D = 21.5;
        public static double F = 15.3;  // Feed-forward
    }

    @Configurable
    public static class ShooterControl {
        public static double maxVelocity = 2360.0;
        public static double velocityTolerance = 100.0;
        public static double trimStep = 100.0;
    }

    @Configurable
    public static class FeederTiming {
        public static long feedPulseMS = 150;        // How long each shot feeds for
        public static long feedCooldownMS = 500;     // Delay between shots
    }

    @Configurable
    public static class LightIndicator {
        public static double greenPosition = 0.42;   // At speed (stable for 2 sec)
        public static double redPosition = 0.30;     // Spinning up
        public static double bluePosition = 0.60;    // Off
    }

    @Configurable
    public static class DistanceVelocityMap {
        public static double distance1 = 20.0;
        public static double velocity1 = 1200.0;
        public static double distance2 = 40.0;
        public static double velocity2 = 1700.0;
        public static double distance3 = 60.0;
        public static double velocity3 = 2100.0;
        public static double distance4 = 80.0;
        public static double velocity4 = 2360.0;
    }

    @Configurable
    public static class TagAlignmentControl {
        public static double alignmentDeadbandDeg = 0.7;
        public static double headingPID_P = 0.02;  // Tune rotation response
    }

    // ========== HARDWARE NAMES ==========
    private static final String SHOOTER_NAME = "shooter";
    private static final String SHOOTER2_NAME = "shooter2";

    private static final String FEED_LEFT_NAME = "feedLeft";
    private static final String FEED_RIGHT_NAME = "feedRight";

    private static final String LIGHT_NAME = "shooterLight";

    // ========== MOTOR/SERVO INSTANCES ==========
    private DcMotorEx shooter;
    private DcMotorEx shooter2;

    private Limelight3A limelight;
    private Follower follower;

    private CRServo feedLeft, feedRight;
    private Servo shooterLight;

    // ========== GOAL POSITIONS ==========
    private static final Pose BLUE_GOAL = new Pose(24.0, 123.0);
    private static final Pose RED_GOAL = new Pose(122.0, 123.0);

    // ========== SHOOTER VARIABLES ==========
    private AbstractBijectiveMap.NumericBijectiveMap distVelMap;
    private double velocityTrimOffset = 0.0;

    private long shooterAtSpeedTime = 0;
    private boolean wasAtSpeed = false;
    private static final long SPEED_STABLE_DURATION = 2000; // milliseconds

    // Bumper edge detection
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    boolean shooterOn = false;

    // ========== FEEDER STATE MACHINE ==========
    private enum FeederState {
        IDLE,
        FEEDING,
        COOLDOWN
    }

    private FeederState feederState = FeederState.IDLE;
    private long feederStateStartTime = 0;

    // ========== TAG ALIGNMENT VARIABLES ==========
    private boolean alignmentActive = false;
    private double targetHeading = 0;

    // Alliance selection — toggles with gamepad1.start
    // true = Red (tag 24), false = Blue (tag 20)
    private boolean isRedAlliance = true;
    private static final int RED_TAG_ID = 24;
    private static final int BLUE_TAG_ID = 20;

    @Override
    public void init() {

        // Initialize feeder timer
        feederStateStartTime = System.currentTimeMillis();

        // Create Pedro Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        // ========== SHOOTER ==========
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, SHOOTER2_NAME);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients from config group
        shooter.setVelocityPIDFCoefficients(ShooterPID.P, ShooterPID.I, ShooterPID.D, ShooterPID.F);

        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        // ========== LIGHT INDICATOR ==========
        shooterLight = hardwareMap.get(Servo.class, LIGHT_NAME);

        // ========== FEEDER SERVOS ==========
        feedLeft = hardwareMap.get(CRServo.class, FEED_LEFT_NAME);
        feedRight = hardwareMap.get(CRServo.class, FEED_RIGHT_NAME);

        feedLeft.setDirection(CRServo.Direction.FORWARD);
        feedRight.setDirection(CRServo.Direction.FORWARD);

        // ========== LIMELIGHT ==========
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        // ========== DISTANCE-VELOCITY MAP ==========
        distVelMap = new AbstractBijectiveMap.NumericBijectiveMap();
        distVelMap.put(DistanceVelocityMap.distance1, DistanceVelocityMap.velocity1);
        distVelMap.put(DistanceVelocityMap.distance2, DistanceVelocityMap.velocity2);
        distVelMap.put(DistanceVelocityMap.distance3, DistanceVelocityMap.velocity3);
        distVelMap.put(DistanceVelocityMap.distance4, DistanceVelocityMap.velocity4);

        telemetry.addLine("Pedro Pathing + Shooter + Limelight + Feeder Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        // Required by Pedro Pathing before you can call setTeleOpDrive in loop()
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // Update PIDF coefficients from config (allows real-time tuning)
        shooter.setVelocityPIDFCoefficients(ShooterPID.P, ShooterPID.I, ShooterPID.D, ShooterPID.F);

        // ---------- DRIVETRAIN with Pedro Pathing ----------
        double y = -gamepad1.left_stick_y;   // Forward/backward
        double x = -gamepad1.left_stick_x;   // Strafe left/right
        double rx = -gamepad1.right_stick_x;  // Rotation (used when alignment OFF)

        // ---------- ALLIANCE TOGGLE (Start Button) ----------
        if (gamepad1.startWasPressed()) {
            isRedAlliance = !isRedAlliance;
        }
        int targetTagId = isRedAlliance ? RED_TAG_ID : BLUE_TAG_ID;

        double currentHeading = follower.getPose().getHeading();

        // ---------- TAG ALIGNMENT TOGGLE (A Button) ----------
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

                        if (Math.abs(tx) > TagAlignmentControl.alignmentDeadbandDeg) {
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
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            double rotationOutput = Math.max(-1.0, Math.min(1.0,
                    TagAlignmentControl.headingPID_P * headingError));

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

        // ---------- SHOOTER VELOCITY TRIM (Bumpers fine-tune offset) ----------
        if (gamepad1.right_bumper && !lastRightBumper) {
            velocityTrimOffset += ShooterControl.trimStep;
        }
        if (gamepad1.left_bumper && !lastLeftBumper) {
            velocityTrimOffset -= ShooterControl.trimStep;
        }

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        // ---------- DISTANCE-BASED TARGET VELOCITY ----------
        Pose currentPose = follower.getPose();
        Pose goal = isRedAlliance ? RED_GOAL : BLUE_GOAL;
        double distanceToGoal = Math.max(0, Math.hypot(currentPose.getX() - goal.getX(), currentPose.getY() - goal.getY()) - 4.0);

        double targetVelocity = MathFunctions.clamp(
                getVelocityForDistance(distanceToGoal) + velocityTrimOffset,
                0, ShooterControl.maxVelocity);

        // ---------- SHOOTER SPEED LIGHT CONTROL ----------
        double currentVelocity = shooter.getVelocity();
        boolean atSpeed = shooterOn &&
                Math.abs(currentVelocity) >= (targetVelocity - ShooterControl.velocityTolerance) &&
                Math.abs(currentVelocity) <= (targetVelocity + ShooterControl.velocityTolerance);

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
            shooterLight.setPosition(LightIndicator.greenPosition);  // green - stable at speed
        } else if (shooterOn) {
            shooterLight.setPosition(LightIndicator.redPosition);    // red - spinning up
        } else {
            shooterLight.setPosition(LightIndicator.bluePosition);   // blue - off
        }

        // Toggle shooter (B button)
        if (gamepad1.bWasPressed()) {
            shooterOn = !shooterOn;
        }

        // Apply shooter state
        if (shooterOn) {
            shooter.setVelocity(targetVelocity);
            shooter2.setPower(targetVelocity / ShooterControl.maxVelocity); // no encoder, scale to 0-1
        } else {
            shooter.setVelocity(0);
            shooter2.setPower(0);
        }

        // ---------- FEEDER STATE MACHINE (Hold Y to fire continuously) ----------
        long currentTime = System.currentTimeMillis();
        long elapsedTime = currentTime - feederStateStartTime;

        switch (feederState) {
            case IDLE:
                feedLeft.setPower(0);
                feedRight.setPower(0);

                if (gamepad1.y) {
                    feederState = FeederState.FEEDING;
                    feederStateStartTime = currentTime;
                } else if (gamepad1.x) {
                    // Reverse feed
                    feedLeft.setPower(-1.0);
                    feedRight.setPower(1.0);
                }
                break;

            case FEEDING:
                feedLeft.setPower(1.0);
                feedRight.setPower(-1.0);

                if (elapsedTime >= FeederTiming.feedPulseMS) {
                    feederState = FeederState.COOLDOWN;
                    feederStateStartTime = currentTime;
                }

                if (!gamepad1.y) {
                    feederState = FeederState.IDLE;
                }
                break;

            case COOLDOWN:
                feedLeft.setPower(0);
                feedRight.setPower(0);

                if (!gamepad1.y) {
                    feederState = FeederState.IDLE;
                } else if (elapsedTime >= FeederTiming.feedCooldownMS) {
                    feederState = FeederState.FEEDING;
                    feederStateStartTime = currentTime;
                }
                break;
        }

        // ---------- DEBUG TELEMETRY ----------
        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Distance to Goal (in)", String.format("%.1f", distanceToGoal));
        telemetry.addData("Target Velocity (t/s)", String.format("%.0f", targetVelocity));
        telemetry.addData("Current Velocity (t/s)", String.format("%.0f", currentVelocity));
        telemetry.addData("Velocity Trim Offset", velocityTrimOffset);
        telemetry.addData("At Speed", atSpeed);
        telemetry.addData("PIDF", String.format("P=%.1f I=%.1f D=%.1f F=%.1f", ShooterPID.P, ShooterPID.I, ShooterPID.D, ShooterPID.F));

        telemetry.addData("=== FEEDER ===", "");
        telemetry.addData("Feeder State", feederState.toString());
        telemetry.addData("Feeder L Power", feedLeft.getPower());
        telemetry.addData("Feeder R Power", feedRight.getPower());

        telemetry.addData("=== PEDRO PATHING ===", "");
        telemetry.addData("Current Pose", follower.getPose());
        telemetry.addData("Current Heading (deg)", String.format("%.1f", Math.toDegrees(currentHeading)));
        telemetry.addData("Target Heading (deg)", String.format("%.1f", Math.toDegrees(targetHeading)));

        telemetry.addData("=== TAG ALIGNMENT ===", "");
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
                    telemetry.addData("TX (deg)", String.format("%.1f", fiducial.getTargetXDegrees()));
                    telemetry.addData("TY (deg)", String.format("%.1f", fiducial.getTargetYDegrees()));
                    telemetry.addData("TA (%)", String.format("%.1f", fiducial.getTargetArea()));
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