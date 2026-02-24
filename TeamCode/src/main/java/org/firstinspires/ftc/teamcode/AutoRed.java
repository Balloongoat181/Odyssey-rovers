package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoRed extends OpMode {
    enum AutoState {
        BACKUP, WAIT_FOR_SPINUP, SHOOTONE, OFFONE,
        SHOOTWO, OFFTWO,
        SHOOTTHREE, OFFTHREE,
        SHOOTFOUR, OFFALL, MOVERIGHT, TURNLEFT, MOVEFORWARD
    }

    ElapsedTime timer = new ElapsedTime();

    private static final String FL_NAME = "frontLeft";
    private static final String FR_NAME = "frontRight";
    private static final String BL_NAME = "backLeft";
    private static final String BR_NAME = "backRight";

    private static final String SHOOTER_NAME = "shooter";
    private static final String FEED_LEFT_NAME = "feedLeft";
    private static final String FEED_RIGHT_NAME = "feedRight";

    private DcMotorEx fl, fr, bl, br;
    private DcMotorEx shooter;
    private CRServo feedLeft, feedRight;
    private AutoState currentState = AutoState.BACKUP;

    @Override
    public void init() {

        // Initialize hardware
        fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
        bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        br = hardwareMap.get(DcMotorEx.class, BR_NAME);

        shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        feedLeft = hardwareMap.get(CRServo.class, FEED_LEFT_NAME);
        feedRight = hardwareMap.get(CRServo.class, FEED_RIGHT_NAME);

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
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);


    }

    @Override
    public void init_loop() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }


    @Override
    public void loop() {
        switch (currentState) {
            case BACKUP:
                shooter.setPower(0.555);
                fl.setPower(-0.5);
                fr.setPower(-0.5);
                bl.setPower(-0.5);
                br.setPower(-0.5);
                if (timer.milliseconds() > 1400) {
                    currentState = AutoState.WAIT_FOR_SPINUP;
                    timer.reset();
                }
                break;
            case WAIT_FOR_SPINUP:
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);

                if (timer.milliseconds() > 5000) {
                    currentState = AutoState.SHOOTONE;
                    timer.reset();
                }
                break;
            case SHOOTONE:
                feedLeft.setPower(1.0);
                feedRight.setPower(-1.0);
                if (timer.milliseconds() > 150) {
                    currentState = AutoState.OFFONE;
                    timer.reset();
                }
                break;
            case OFFONE:
                feedLeft.setPower(0);
                feedRight.setPower(0);
                if (timer.milliseconds() > 3000) {
                    currentState = AutoState.SHOOTWO;
                    timer.reset();
                }
                break;
            case SHOOTWO:
                feedLeft.setPower(1.0);
                feedRight.setPower(-1.0);
                if (timer.milliseconds() > 150) {
                    currentState = AutoState.OFFTWO;
                    timer.reset();
                }
                break;
            case OFFTWO:
                feedLeft.setPower(0);
                feedRight.setPower(0);
                if (timer.milliseconds() > 3000) {
                    currentState = AutoState.SHOOTTHREE;
                    timer.reset();
                }
                break;
            case SHOOTTHREE:
                feedLeft.setPower(1.0);
                feedRight.setPower(-1.0);
                if (timer.milliseconds() > 150) {
                    currentState = AutoState.OFFTHREE;
                    timer.reset();
                }
                break;
            case OFFTHREE:
                feedLeft.setPower(0);
                feedRight.setPower(0);
                if (timer.milliseconds() > 3000) {
                    currentState = AutoState.SHOOTFOUR;
                    timer.reset();
                }
                break;
            case SHOOTFOUR:
                feedLeft.setPower(1.0);
                feedRight.setPower(-1.0);
                if (timer.milliseconds() > 1000) {
                    currentState = AutoState.TURNLEFT;
                    timer.reset();

                }
                break;
            case TURNLEFT:
                feedLeft.setPower(0);
                feedRight.setPower(0);
                shooter.setPower(0);
                fl.setPower(-0.5);
                fr.setPower(0.5);
                bl.setPower(-0.5);
                br.setPower(0.5);
                if (timer.milliseconds() > 500) {
                    currentState = AutoState.MOVEFORWARD;
                    timer.reset();
                }
                break;
            case MOVEFORWARD:
                fl.setPower(0.5);
                fr.setPower(0.5);
                bl.setPower(0.5);
                br.setPower(0.5);
                if (timer.milliseconds() > 700) {
                    currentState = AutoState.OFFALL;
                    timer.reset();
                }
                break;
            case OFFALL:
                feedLeft.setPower(0);
                feedRight.setPower(0);
                shooter.setPower(0);
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                // currentState = AutoState.MOVERIGHT;
                break;
        }
            /*case MOVERIGHT:
                feedLeft.setPower(0);
                feedRight.setPower(0);
                shooter.setPower(0);
                fl.setPower(0.5);
                fr.setPower(-0.5);
                bl.setPower(-0.5);
                br.setPower(0.5);
                if (timer.milliseconds() > 600) {
                    fl.setPower(0);
                    fr.setPower(0);
                    bl.setPower(0);
                    br.setPower(0);
                }
                break;





        }
    /*
            // Drive forward for 1 second
            fl.setPower(-0.5);
            fr.setPower(-0.5);
            bl.setPower(-0.5);
            br.setPower(-0.5);

            sleep(1400);

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            
            shooter.setPower(0.555);
            
            sleep(5000);
            
            feedLeft.setPower(1.0);
            feedRight.setPower(-1.0);
            
            sleep(1000);
            
            feedLeft.setPower(0);
            feedRight.setPower(0);
            
            sleep(3000);
            
            feedLeft.setPower(1.0);
            feedRight.setPower(-1.0);
            
            sleep(1000);
             
            feedLeft.setPower(0);
            feedRight.setPower(0);
            
            sleep(3000);
            
            feedLeft.setPower(1.0);
            feedRight.setPower(-1.0);
            
            sleep(1000);
            
            
            feedLeft.setPower(0);
            feedRight.setPower(0);
            sleep(3000);
            feedLeft.setPower(1.0);
            feedRight.setPower(-1.0);
            sleep(1000);
            feedLeft.setPower(0);
            feedRight.setPower(0);
            shooter.setPower(0);
            */


        telemetry.addData("Status", "Working");
        telemetry.update();
    }
}





