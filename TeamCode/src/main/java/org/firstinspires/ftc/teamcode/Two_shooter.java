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

    // PANELS WILL SEE THESE SLIDERS NOW
    public static double P = 23;
    public static double I = 0.0;
    public static double D = 21.5;
    public static double F = 15.3; // Your current feed-forward
    private double shooterVelocity = 1300; // FIXED: typo was "shooterVeloicty"


    private static final String FL_NAME = "frontLeft";
    private static final String FR_NAME = "frontRight";
    private static final String BL_NAME = "backLeft";
    private static final String BR_NAME = "backRight";

    private static final String SHOOTER_NAME = "shooter";

    private static final String SHOOTER2_NAME= "shooter2";

    // Gecko feed servos
    private static final String FEED_LEFT_NAME  = "feedLeft";
    private static final String FEED_RIGHT_NAME = "feedRight";

    // Light indicator
    private static final String LIGHT_NAME = "shooterLight";

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx shooter;

    private DcMotorEx shooter2;

    private CRServo feedLeft, feedRight;
    private Servo shooterLight;

    // Shooter adjustable power

    // Maximum velocity your shooter can reach at full power (adjust based on testing)
    private static final double MAX_VELOCITY = 2800; // ticks per second at 100% power
    private static final double VELOCITY_TOLERANCE = 100.0; // tolerance range

    // Bumper edge detection
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    boolean shooterOn = false;

    // Feeder state machine timing
    private long feederStartTime = 0;
    private static final long FEED_DURATION_MS = 150;
    private boolean isFeedingForward = false;

    @Override
    public void init() {

        // Drivetrain
        fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
        bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        br = hardwareMap.get(DcMotorEx.class, BR_NAME);

        // Shooter
        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, SHOOTER2_NAME);

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
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Motor modes
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset and enable shooter encoder
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setVelocityPIDFCoefficients(P, I, D, F);

        // Motor directions
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);


        // Servo directions — adjust if spinning wrong
        feedLeft.setDirection(CRServo.Direction.FORWARD);
        feedRight.setDirection(CRServo.Direction.FORWARD);

        telemetry.addLine("Mecanum + Shooter + Feeder Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        shooter.setVelocityPIDFCoefficients(P, I, D, F);

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
        if (gamepad1.right_bumper && !lastRightBumper) shooterVelocity += 50;
        if (gamepad1.left_bumper && !lastLeftBumper) shooterVelocity -= 50;

        shooterVelocity = Math.max(0.0, Math.min(MAX_VELOCITY, shooterVelocity));

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        // Toggle shooter
        if (gamepad1.bWasPressed()) {
            shooterOn = !shooterOn;
        }

        // Apply shooter state
        if (shooterOn) {
            shooter.setVelocity(shooterVelocity);
            shooter2.setPower(shooterVelocity/MAX_VELOCITY);
        } else {
            shooter.setVelocity(0);
            shooter2.setPower(0);
        }

        // ---------- Shooter Speed Light Control ----------
        double targetVelocity = shooterVelocity;
        double currentVelocity = shooter.getVelocity();

        // FIXED: Velocity must be within tolerance on BOTH sides (upper and lower bounds)
        boolean atSpeed = shooterOn &&
                Math.abs(currentVelocity) >= (targetVelocity - VELOCITY_TOLERANCE) &&
                Math.abs(currentVelocity) <= (targetVelocity + VELOCITY_TOLERANCE);

        // Set light color based on state
        if (atSpeed) {
            shooterLight.setPosition(0.42);  // green - stable at speed for 2 seconds
        } else if (shooterOn) {
            shooterLight.setPosition(0.30);  // red - shooter on but not at speed yet
        } else {
            shooterLight.setPosition(0.60);  // blue - shooter off
        }

        // ---------- Gecko Feed Servos (Non-blocking state machine) ----------
        // FIXED: Removed blocking sleep() call; replaced with timer-based approach
        long currentTime = System.currentTimeMillis();

        // If a forward feed is active and time has elapsed, stop it
        if (isFeedingForward && (currentTime >= FEED_DURATION_MS)) {
            isFeedingForward = false;
            feedLeft.setPower(0);
            feedRight.setPower(0);
        }

        if (gamepad1.y) {
            // Start forward feed
            feedLeft.setPower(1.0);
            feedRight.setPower(-1.0);
            feederStartTime = currentTime;
            isFeedingForward = true;
        }
        else if (gamepad1.x) {
            // Reverse feed (Xbox/Logitech X button)
            feedLeft.setPower(-1.0);
            feedRight.setPower(1.0);
            isFeedingForward = false;
        }
        else if (!isFeedingForward) {
            // Only stop if not in the middle of a timed forward feed
            feedLeft.setPower(0);
            feedRight.setPower(0);
        }

        // ---------- Debug Telemetry ----------
        telemetry.addData("Y pressed", gamepad1.y);
        telemetry.addData("X pressed", gamepad1.x);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Set Velocity", shooterVelocity); // FIXED: typo
        telemetry.addData("Velocity", shooter.getVelocity()); // FIXED: typo
        telemetry.addData("At Speed", atSpeed);
        telemetry.addData("Feeder L Power", feedLeft.getPower());
        telemetry.addData("Feeder R Power", feedRight.getPower());
        telemetry.update();
    }
}