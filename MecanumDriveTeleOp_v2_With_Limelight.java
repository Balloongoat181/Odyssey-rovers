package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Mecanum Drive TeleOp", group = "Drive")
public class MecanumDriveTeleOp_v2_With_Limelight extends OpMode {

    private static final String FL_NAME = "frontLeft";
    private static final String FR_NAME = "frontRight";
    private static final String BL_NAME = "backLeft";
    private static final String BR_NAME = "backRight";
    private static final String SHOOTER_NAME = "shooter";
    private static final String FEED_LEFT_NAME = "feedLeft";
    private static final String FEED_RIGHT_NAME = "feedRight";
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

    // AprilTag Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean aprilTagAlignMode = false;
    private boolean dpadDownPrevious = false;

    // Alliance selection (set via DPAD UP at init)
    private int targetGoalTag = 20; // Default Blue
    private static final double ALIGNMENT_TOLERANCE = 2.0; // degrees

    // Shooter variables
    private double shooterPower = 0.6;
    private static final double MAX_VELOCITY = 2360.0;
    private static final double VELOCITY_TOLERANCE = 100.0;
    private long shooterAtSpeedTime = 0;
    private boolean wasAtSpeed = false;
    private static final long SPEED_STABLE_DURATION = 2000;
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
        feedLeft = hardwareMap.get(CRServo.class, FEED_LEFT_NAME);
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

        // Servo directions
        feedLeft.setDirection(CRServo.Direction.FORWARD);
        feedRight.setDirection(CRServo.Direction.FORWARD);

        // Initialize AprilTag detection
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Optimize camera settings
        setManualExposure(5, 100);

        telemetry.addLine("Mecanum + Shooter + AprilTag Ready");
        telemetry.addLine("DPAD_UP at init = Set Alliance");
        telemetry.addLine("DPAD_DOWN in game = Toggle AprilTag Align");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Alliance selection during init
        if (gamepad1.dpad_up) {
            targetGoalTag = 20; // Blue
            telemetry.addLine("✓ BLUE Alliance - Tag 20");
        } else if (gamepad1.dpad_right) {
            targetGoalTag = 24; // Red
            telemetry.addLine("✓ RED Alliance - Tag 24");
        }
        telemetry.addData("Target Goal Tag", targetGoalTag);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower localization
        follower.update();
        // ---------- AprilTag Align Toggle (DPAD_DOWN) ----------
        if (gamepad1.dpad_down && !dpadDownPrevious) {
            aprilTagAlignMode = !aprilTagAlignMode;
            if (!aprilTagAlignMode) {
                // Cancel alignment
                positionLocked = false;
                follower.breakFollowing();
            }
        }
        dpadDownPrevious = gamepad1.dpad_down;

        // ---------- Position Lock Toggle (A Button) ----------
        if (gamepad1.a && !aButtonPrevious) {
            positionLocked = !positionLocked;

            if (positionLocked) {
                targetPose = follower.getPose();
                follower.holdPoint(targetPose);
                aprilTagAlignMode = false; // Disable AprilTag mode
            } else {
                follower.breakFollowing();
            }
        }
        aButtonPrevious = gamepad1.a;

        // ---------- Drivetrain Control ----------
        if (aprilTagAlignMode) {
            // AprilTag alignment mode
            alignToGoalTag();

        } else if (positionLocked) {
            // Pedro position hold mode
            // follower.update() handles holding

        } else {
            // Normal driver control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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
        if (gamepad1.right_bumper && !lastRightBumper) shooterPower += 0.05;
        if (gamepad1.left_bumper && !lastLeftBumper) shooterPower -= 0.05;
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

        if (atSpeed && !wasAtSpeed) {
            shooterAtSpeedTime = System.currentTimeMillis();
        }
        if (!atSpeed) {
            shooterAtSpeedTime = 0;
        }
        wasAtSpeed = atSpeed;

        if (atSpeed && (System.currentTimeMillis() - shooterAtSpeedTime >= SPEED_STABLE_DURATION)) {
            shooterLight.setPosition(0.42);  // green
        } else if (shooterOn) {
            shooterLight.setPosition(0.30);  // red
        } else {
            shooterLight.setPosition(0.60);  // blue
        }

        // ---------- Gecko Feed Servos ----------
        if (gamepad1.y) {
            feedLeft.setPower(1.0);
            feedRight.setPower(-1.0);
        } else if (gamepad1.x) {
            feedLeft.setPower(-1.0);
            feedRight.setPower(1.0);
        } else {
            feedLeft.setPower(0);
            feedRight.setPower(0);
        }

        // ---------- Telemetry ----------
        telemetry.addData("AprilTag Align", aprilTagAlignMode ? "ACTIVE" : "Off");
        telemetry.addData("Position Locked", positionLocked);
        telemetry.addData("Target Goal Tag", targetGoalTag);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.update();
    }

    /**
     * Align to goal using AprilTag detection
     */
    private void alignToGoalTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetGoalTag) {
                double yaw = detection.ftcPose.yaw; // Horizontal offset

                telemetry.addData("Tag Found", detection.id);
                telemetry.addData("Yaw Offset", "%.2f°", yaw);

                // Check if aligned
                if (Math.abs(yaw) < ALIGNMENT_TOLERANCE) {
                    // ALIGNED - stop motors
                    fl.setPower(0);
                    fr.setPower(0);
                    bl.setPower(0);
                    br.setPower(0);
                    gamepad1.rumble(200); // Haptic feedback
                    telemetry.addLine("ALIGNED!");
                    return;
                }

                // Apply turning correction
                double turnPower = yaw * 0.015; // Tune this P gain
                turnPower = Math.max(-0.25, Math.min(0.25, turnPower));

                fl.setPower(-turnPower);
                fr.setPower(turnPower);
                bl.setPower(-turnPower);
                br.setPower(turnPower);

                return;
            }
        }

        // Tag not visible
        telemetry.addLine("Goal Tag Not Visible");
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}
