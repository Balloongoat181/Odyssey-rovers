package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Two shooter test", group = "Drive")
public class Two_shooter extends OpMode {

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
        public static double initialVelocity = 1300;
        public static double maxVelocity = 2800;
        public static double velocityTolerance = 100.0;
        public static double velocityIncrement = 50;
    }

    @Configurable
    public static class FeederTiming {
        public static long feedPulseMS = 150;        // How long each shot feeds for
        public static long feedCooldownMS = 300;     // Delay between shots
    }

    @Configurable
    public static class LightIndicator {
        public static double greenPosition = 0.42;   // At speed
        public static double redPosition = 0.30;     // Spinning up
        public static double bluePosition = 0.60;    // Off
    }

    // ========== HARDWARE NAMES ==========
    private static final String FL_NAME = "frontLeft";
    private static final String FR_NAME = "frontRight";
    private static final String BL_NAME = "backLeft";
    private static final String BR_NAME = "backRight";

    private static final String SHOOTER_NAME = "shooter";
    private static final String SHOOTER2_NAME = "shooter2";

    private static final String FEED_LEFT_NAME = "feedLeft";
    private static final String FEED_RIGHT_NAME = "feedRight";

    private static final String LIGHT_NAME = "shooterLight";

    // ========== MOTOR/SERVO INSTANCES ==========
    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx shooter;
    private DcMotorEx shooter2;

    private CRServo feedLeft, feedRight;
    private Servo shooterLight;

    // ========== RUNTIME VARIABLES ==========
    private double shooterVelocity;
    private boolean shooterOn = false;

    // Bumper edge detection
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    // Feeder state machine
    private enum FeederState {
        IDLE,
        FEEDING,
        COOLDOWN
    }

    private FeederState feederState = FeederState.IDLE;
    private long feederStateStartTime = 0;

    @Override
    public void init() {
        // Initialize shooter velocity from config
        shooterVelocity = ShooterControl.initialVelocity;

        // Initialize timer
        feederStateStartTime = System.currentTimeMillis();

        // ========== DRIVETRAIN ==========
        fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
        bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        br = hardwareMap.get(DcMotorEx.class, BR_NAME);

        // ========== SHOOTER ==========
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, SHOOTER2_NAME);

        // ========== FEEDER SERVOS ==========
        feedLeft = hardwareMap.get(CRServo.class, FEED_LEFT_NAME);
        feedRight = hardwareMap.get(CRServo.class, FEED_RIGHT_NAME);

        // ========== LIGHT INDICATOR ==========
        shooterLight = hardwareMap.get(Servo.class, LIGHT_NAME);

        // ========== ZERO POWER BEHAVIOR ==========
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ========== MOTOR MODES ==========
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // ========== PIDF COEFFICIENTS ==========
        shooter.setVelocityPIDFCoefficients(ShooterPID.P, ShooterPID.I, ShooterPID.D, ShooterPID.F);

        // ========== MOTOR DIRECTIONS ==========
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        // ========== SERVO DIRECTIONS ==========
        feedLeft.setDirection(CRServo.Direction.FORWARD);
        feedRight.setDirection(CRServo.Direction.FORWARD);

        telemetry.addLine("Mecanum + Shooter + Feeder Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update PIDF coefficients from config
        shooter.setVelocityPIDFCoefficients(ShooterPID.P, ShooterPID.I, ShooterPID.D, ShooterPID.F);

        // ---------- DRIVETRAIN ----------
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

        // ---------- SHOOTER VELOCITY ADJUST ----------
        if (gamepad1.right_bumper && !lastRightBumper) {
            shooterVelocity += ShooterControl.velocityIncrement;
        }
        if (gamepad1.left_bumper && !lastLeftBumper) {
            shooterVelocity -= ShooterControl.velocityIncrement;
        }

        shooterVelocity = Math.max(0.0, Math.min(ShooterControl.maxVelocity, shooterVelocity));

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        // Toggle shooter on/off
        if (gamepad1.bWasPressed()) {
            shooterOn = !shooterOn;
        }

        // Apply shooter state
        if (shooterOn) {
            shooter.setVelocity(shooterVelocity);
            shooter2.setPower(shooterVelocity / ShooterControl.maxVelocity);
        } else {
            shooter.setVelocity(0);
            shooter2.setPower(0);
        }

        // ---------- SHOOTER SPEED LIGHT CONTROL ----------
        double currentVelocity = shooter.getVelocity();
        boolean atSpeed = shooterOn &&
                Math.abs(currentVelocity) >= (shooterVelocity - ShooterControl.velocityTolerance) &&
                Math.abs(currentVelocity) <= (shooterVelocity + ShooterControl.velocityTolerance);

        if (atSpeed) {
            shooterLight.setPosition(LightIndicator.greenPosition);
        } else if (shooterOn) {
            shooterLight.setPosition(LightIndicator.redPosition);
        } else {
            shooterLight.setPosition(LightIndicator.bluePosition);
        }

        // ---------- FEEDER STATE MACHINE (Hold Y to fire) ----------
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
        telemetry.addData("Y pressed", gamepad1.y);
        telemetry.addData("X pressed", gamepad1.x);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Set Velocity", shooterVelocity);
        telemetry.addData("Current Velocity", shooter.getVelocity());
        telemetry.addData("At Speed", atSpeed);
        telemetry.addData("Feeder State", feederState.toString());
        telemetry.addData("Feeder L Power", feedLeft.getPower());
        telemetry.addData("Feeder R Power", feedRight.getPower());
        telemetry.update();
    }
}