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
    private double shooterVelocity = 1300;

    // FEEDER TIMING - TUNABLE VIA CONFIG PANEL
    public static long FEED_PULSE_MS = 150;      // How long each shot feeds for
    public static long FEED_COOLDOWN_MS = 300;   // Delay between shots
    public static long STABLE_SPEED = 500;

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

    // Maximum velocity your shooter can reach at full power (adjust based on testing)
    private static final double MAX_VELOCITY = 2800; // ticks per second at 100% power
    private static final double VELOCITY_TOLERANCE = 100.0; // tolerance range

    // Bumper edge detection
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    boolean shooterOn = false;

    // Feeder state machine
    private enum FeederState {
        IDLE,       // Waiting for input
        FEEDING,    // Actively feeding (servo running)
        COOLDOWN    // Waiting between shots
    }

    private FeederState feederState = FeederState.IDLE;
    private long feederStateStartTime = 0;

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

        // Velocity must be within tolerance on BOTH sides (upper and lower bounds)
        boolean atSpeed = shooterOn &&
                Math.abs(currentVelocity) >= (targetVelocity - VELOCITY_TOLERANCE) &&
                Math.abs(currentVelocity) <= (targetVelocity + VELOCITY_TOLERANCE);

        // Set light color based on state
        if (atSpeed) {
            shooterLight.setPosition(0.42);  // green - stable at speed
        } else if (shooterOn) {
            shooterLight.setPosition(0.30);  // red - shooter on but not at speed yet
        } else {
            shooterLight.setPosition(0.60);  // blue - shooter off
        }

        // ---------- Gecko Feed Servos (Hold Y for continuous fire with delays) ----------
        long currentTime = System.currentTimeMillis();
        long elapsedTime = currentTime - feederStateStartTime;

        // STATE MACHINE for feeder
        switch (feederState) {
            case IDLE:
                // Waiting for input
                feedLeft.setPower(0);
                feedRight.setPower(0);

                if (gamepad1.y) {
                    // User pressed Y - start feeding
                    feederState = FeederState.FEEDING;
                    feederStateStartTime = currentTime;
                }
                else if (gamepad1.x) {
                    // Reverse feed (continuous while held)
                    feedLeft.setPower(-1.0);
                    feedRight.setPower(1.0);
                }
                break;

            case FEEDING:
                // Currently feeding - run servos for pulse duration
                feedLeft.setPower(1.0);
                feedRight.setPower(-1.0);

                if (elapsedTime >= FEED_PULSE_MS) {
                    // Pulse complete - go to cooldown
                    feederState = FeederState.COOLDOWN;
                    feederStateStartTime = currentTime;
                }

                // If user released Y, stop immediately
                if (!gamepad1.y) {
                    feederState = FeederState.IDLE;
                }
                break;

            case COOLDOWN:
                // Waiting between shots - servos off
                feedLeft.setPower(0);
                feedRight.setPower(0);

                if (!gamepad1.y) {
                    // User released Y - return to idle
                    feederState = FeederState.IDLE;
                }
                else if (elapsedTime >= FEED_COOLDOWN_MS) {
                    // Cooldown complete - fire again if Y still held
                    feederState = FeederState.FEEDING;
                    feederStateStartTime = currentTime;
                }
                break;
        }

        // ---------- Debug Telemetry ----------
        telemetry.addData("Y pressed", gamepad1.y);
        telemetry.addData("X pressed", gamepad1.x);
        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Set Velocity", shooterVelocity);
        telemetry.addData("Current Velocity", shooter.getVelocity());
        telemetry.addData("At Speed", atSpeed);
        telemetry.addData("Feeder State", feederState.toString());
        telemetry.addData("Feed Pulse MS", FEED_PULSE_MS);
        telemetry.addData("Cooldown MS", FEED_COOLDOWN_MS);
        telemetry.addData("Feeder L Power", feedLeft.getPower());
        telemetry.addData("Feeder R Power", feedRight.getPower());
        telemetry.update();
    }
}