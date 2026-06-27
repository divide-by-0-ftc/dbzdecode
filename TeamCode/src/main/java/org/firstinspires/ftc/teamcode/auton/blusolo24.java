package org.firstinspires.ftc.teamcode.auton;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;


@Config
@Autonomous(name = "blue24flow")
public class blusolo24 extends DbzOpMode
{

    public static double servooff  = 0.01;
    public static double push0     = 0.81,  push3    = 0.22;
    public static double lockpos   = 0.71;
    public static double holdopen  = 0.8,   holdclose = 0.467;



    public static double intakewaittimeout = 1.0;
    public static double revdebounce       = 1.5;
    public static double lockdebounce      = 1.4;
    public static double sticky            = 0.15;



    public static double bangff = 0.85;



    public static double dipamt   = 0.2;
    public static double dipdelay = 0.7;
    public static double dipdur   = 0.15;



    public static double goalx = 143, goaly = 140;
    public static double timeA = 0.00002, timeB = 0.004, timeC = 0.25;



    public static double tzero  = 181;
    public static double tkp    = 0.02, tki = 0.0, tkd = 0.001;
    public static double tdead  = 0.0,  tmax = 1.0, tks = 0.0, tffdead = 0.0;
    public static double thresh = 220,  thresh2 = 220;



    public static double startx =  35.33372228704784, starty = 133.472;
    public static double tG = 0.33,  tS = 0.10;


    public static double gatex =  13, gatey = 60.67, gateh = -158.75;


    public static double secondspikeshootx= 54, secondspikeshooty= 81.00;
    public static double shootx =  57.85, shooty = 82.0;


    public static double nearwallx =  126.373, nearwally = 84.566;



    public static double dthresh  = 0.157, dthresh1 = 0.173, dthresh2 = 0.155;




    public static double neargatex =  17.0, neargatey = 61.55;



    public static double hood1  = 0.50, vel1  = 1500, turret1  = -158;
    public static double hood3  = 0.6, vel3  = 1610, turret3  = 139;
    public static double hood5  = 0.6, vel5  = 1595, turret5  = 104.5;
    public static double hood7  = 0.6, vel7  = 1580, turret7  = 109.5;
    public static double hood9  = 0.6, vel9  = 1580, turret9  = 109.5;
    public static double hood11 = 0.6, vel11 = 1580, turret11 = 109.5;
    public static double hood13 = 0.6, vel13 = 1580, turret13 = 109.5;

    public static double hood15 = 0.6, vel15 = 1450, turret15 = 104.5;



    public static double hoodrest   = 0.6, velrest   = 1520, turretrest = 109.5;



    public static double parkx =  23.5, parky = shootx;







    public static double sotmBias = 10.0;







    public static class Paths
    {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7,
                Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16;


        public Paths(Follower f)
        {

            Path1 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(startx, starty), new Pose( 47.6, 86.5)))
                    .setTangentHeadingInterpolation().setReversed().build();



            Path2 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose( 47.6, 86.5), new Pose( 21, 86.5)))
                    .setTangentHeadingInterpolation().build();



            Path3 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose( 15, 86.5), new Pose( 57, 74)))
                    .setTangentHeadingInterpolation().setReversed().build();



            Path4 = f.pathBuilder()
                    .addPath(new BezierCurve(new Pose( 57, 74),
                            new Pose((144-110.990), 61.89961), new Pose( 16.9, 65.19)))
                    .setTangentHeadingInterpolation().build();



            Path5 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose( 17.0, 61.4), new Pose(secondspikeshootx, secondspikeshooty)))
                    .setTangentHeadingInterpolation().setReversed().build();



            Path6  = gatePath(f, shootx, shooty, gatex, gatey);
            Path7  = returnPath(f, gatex, gatey, shootx, shooty);
            Path8  = gatePath(f, shootx, shooty, gatex, gatey+0.2);
            Path9  = returnPath(f, gatex, gatey+0.2, shootx, shooty);
            Path10 = gatePath(f, shootx, shooty, gatex, gatey+0.2);
            Path11 = returnPath(f, gatex, gatey+0.2, shootx, shooty);
            Path12 = gatePath(f, shootx, shooty, gatex, gatey+0.25);
            Path13 = returnPath(f, gatex, gatey+0.25, shootx, shooty);



            Path14 = gatePath(f, shootx, shooty, gatex, gatey+0.3);



            Path15 = returnPath(f, gatex, gatey+0.3, shootx, shooty);



            Path16 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(shootx, shooty), new Pose(gatex+34, gatey+17)))
                    .setTangentHeadingInterpolation().build();
        }



        private PathChain gatePath(Follower f, double sx, double sy, double gx, double gy)
        {
            return f.pathBuilder()
                    .addPath(new BezierLine(new Pose(sx, sy), new Pose(gx, gy)))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, tG,
                                            HeadingInterpolator.tangent),
                                    new HeadingInterpolator.PiecewiseNode(tG, 1,
                                            HeadingInterpolator.constant(Math.toRadians(-gateh)))
                            )
                    ).build();
        }



        private PathChain returnPath(Follower f, double gx, double gy, double sx, double sy)
        {
            return f.pathBuilder()
                    .addPath(new BezierLine(new Pose(gx, gy), new Pose(sx, sy)))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, tS,
                                            HeadingInterpolator.constant(Math.toRadians(-gateh))),
                                    new HeadingInterpolator.PiecewiseNode(tS, 1,
                                            HeadingInterpolator.tangent.reverse())
                            )
                    ).build();
        }
    }



    protected Servo        rpush, lpush, hood, hold, blinkin;
    protected DcMotorEx    intake, fly1, fly2, turret;
    private   VoltageSensor vsensor;
    private   AnalogInput   tenc, d0, d1, d2;
    private   PIDController tpid;
    private   Follower      follower;
    private   Paths         paths;



    private enum AutonState
    {
        followPath1,  shoot1,
        followPath2,
        followPath3,  shoot3,
        followPath4, gateclear,
        followPath5,  shoot5,
        followPath6,  intakeWait2,
        followPath7,  shoot7,
        followPath8,  intakeWait3,
        followPath9,  shoot9,
        followPath10, intakeWait4,
        followPath11, shoot11,
        followPath12, intakeWait5,
        followPath13, shoot13,

        followPath14, intakeWait6,
        followPath15, shoot15,
        followPath16,
        done
    }
    private AutonState state = AutonState.followPath1;


    private enum BallState { idle, reversing, locked }
    private BallState bstate = BallState.idle;



    private ElapsedTime statetimer  = new ElapsedTime();
    private ElapsedTime revtimer    = new ElapsedTime();
    private ElapsedTime detecttimer = new ElapsedTime();
    private ElapsedTime diptimer    = new ElapsedTime();
    private ElapsedTime st0 = new ElapsedTime();
    private ElapsedTime st1 = new ElapsedTime();
    private ElapsedTime st2 = new ElapsedTime();
    private boolean latch0 = false, latch1 = false, latch2 = false;



    private boolean prevdetect     = false;
    private double  targetvelocity = 0;
    private double  hoodbase       = hoodrest;
    private boolean shooting       = false;
    private boolean dipping        = false, dipdone = false;



    private boolean sotmActive = false;



    @Override
    public void opInit()
    {
        rpush   = hardwareMap.get(Servo.class, "rightpushServo");
        lpush   = hardwareMap.get(Servo.class, "leftpushServo");
        hood    = hardwareMap.get(Servo.class, "hoodServo");
        hold    = hardwareMap.get(Servo.class, "holdServo");
        blinkin = hardwareMap.get(Servo.class, "light");




        d0   = hardwareMap.get(AnalogInput.class, "distancez");
        d1   = hardwareMap.get(AnalogInput.class, "distance1");
        d2   = hardwareMap.get(AnalogInput.class, "distance2");
        tenc = hardwareMap.get(AnalogInput.class, "turretEncoder");


        intake = robot.intakeMotor;
        fly1   = robot.outtake1Motor;
        fly2   = robot.outtake2Motor;
        fly1.setDirection(DcMotorEx.Direction.REVERSE);
        fly2.setDirection(DcMotorEx.Direction.FORWARD);
        fly1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);



        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);


        vsensor = hardwareMap.voltageSensor.iterator().next();


        tpid = new PIDController(tkp, tki, tkd);
        tpid.setTolerance(1.0);


        hood.setPosition(hoodrest);
        hold.setPosition(holdopen);
        lpush.setPosition(lockpos);
        rpush.setPosition(lockpos - servooff);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startx, starty, Math.toRadians(90)));
        paths = new Paths(follower);
        PinpointLocalizer p = (PinpointLocalizer) follower.poseTracker.getLocalizer();
        p.getPinpoint().recalibrateIMU();

        follower.followPath(paths.Path1, true);
        statetimer.reset();
    }



    @Override
    public void opLoop()
    {
        follower.update();
        regressions();
        runflywheel();
        aim();
        dipshot();


        switch (state)
        {

            case followPath1:
                if (follower.getCurrentTValue() > 0.95)
                {
                    intake.setPower(1);
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot1;
                }
                break;


            case shoot1:
                if (statetimer.seconds() >= 0.3)
                {
                    endshoot();
                    follower.followPath(paths.Path2);
                    state = AutonState.followPath2;
                }
                break;



            case followPath2:
                hold.setPosition(holdclose);
                if (follower.getCurrentTValue() > 0.95)
                {
                    follower.followPath(paths.Path3);
                    state = AutonState.followPath3;
                }
                break;


            case followPath3:
                hold.setPosition(holdopen);
                if (follower.getCurrentTValue() > 0.95)
                {
                    intake.setPower(1);
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot3;
                }
                break;


            case shoot3:
                if (statetimer.seconds() >= 0.3)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path4);
                    state = AutonState.followPath4;
                }
                break;



            case followPath4:
                intake.setPower(1);
                runballdetection();
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.gateclear;
                    hold.setPosition(holdopen);
                }
                break;
            case gateclear:
                if (statetimer.seconds() >= 0.2)
                {
                    follower.followPath(paths.Path5);
                    state = AutonState.followPath5;
                }
                break;



            case followPath5:
                if (statetimer.seconds() > 0.3)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                }
                if (statetimer.seconds() >= 0.5)  intake.setPower(-1);
                if (statetimer.seconds() > 0.8)   hold.setPosition(holdopen);
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot5;
                }
                break;


            case shoot5:
                if (statetimer.seconds() >= 0.3)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path6);
                    state = AutonState.followPath6;
                }
                break;



            case followPath6:
                intake.setPower(1);
                runballdetection();
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    hold.setPosition(holdclose);
                    state = AutonState.intakeWait2;
                }
                break;


            case intakeWait2:
                runballdetection();
                if (statetimer.seconds() > 1.6)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                }
                if (statetimer.seconds() >= 2)   intake.setPower(-1);
                if (statetimer.seconds() > 1)    hold.setPosition(holdopen);
                if (bstate == BallState.locked || statetimer.seconds() >= 0.71)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path7);
                    state = AutonState.followPath7;
                }
                break;



            case followPath7:
                if (statetimer.seconds() > lockdebounce)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                }
                if (statetimer.seconds() >= revdebounce)
                {
                    intake.setPower(-1);
                    hold.setPosition(holdopen);
                }
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    intake.setPower(1);
                    statetimer.reset();
                    state = AutonState.shoot7;
                }
                break;


            case shoot7:
                if (statetimer.seconds() >= 0.3)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path8);
                    state = AutonState.followPath8;
                }
                break;



            case followPath8:
                intake.setPower(1);
                hold.setPosition(holdclose);
                runballdetection();
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait3;
                }
                break;


            case intakeWait3:
                runballdetection();
                if (bstate == BallState.locked || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path9);
                    state = AutonState.followPath9;
                }
                break;


            case followPath9:
                if (statetimer.seconds() > lockdebounce)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                }
                if (statetimer.seconds() >= revdebounce)
                {
                    intake.setPower(-1);
                    hold.setPosition(holdopen);
                }
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    intake.setPower(1);
                    statetimer.reset();
                    state = AutonState.shoot9;
                }
                break;


            case shoot9:
                if (statetimer.seconds() >= 0.3)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path10);
                    state = AutonState.followPath10;
                }
                break;



            case followPath10:
                intake.setPower(1);
                hold.setPosition(holdclose);
                runballdetection();
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait4;
                }
                break;


            case intakeWait4:
                runballdetection();
                if (bstate == BallState.locked || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path11);
                    state = AutonState.followPath11;
                }
                break;


            case followPath11:
                if (statetimer.seconds() > lockdebounce)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                }
                if (statetimer.seconds() >= revdebounce)
                {
                    intake.setPower(-1);
                    hold.setPosition(holdopen);
                }
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    intake.setPower(1);
                    statetimer.reset();
                    state = AutonState.shoot11;
                }
                break;


            case shoot11:
                if (statetimer.seconds() >= 0.3)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path12);
                    state = AutonState.followPath12;
                }
                break;



            case followPath12:
                intake.setPower(1);
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait5;
                }
                break;


            case intakeWait5:
                runballdetection();
                if (statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path13);
                    state = AutonState.followPath13;
                }
                break;


            case followPath13:
                hold.setPosition(holdopen);
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot13;
                }
                break;


            case shoot13:
                if (statetimer.seconds() >= 0.3)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path14);
                    state = AutonState.followPath14;
                }
                break;



            case followPath14:
                intake.setPower(1);
                hold.setPosition(holdclose);
                runballdetection();
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait6;
                }
                break;


            case intakeWait6:
                runballdetection();
                if (bstate == BallState.locked || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate     = BallState.idle;
                    prevdetect = false;
                    sotmActive = true;
                    statetimer.reset();
                    follower.followPath(paths.Path15);
                    state = AutonState.followPath15;
                }
                break;



            case followPath15:

                if (statetimer.seconds() > lockdebounce)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                }
                if (statetimer.seconds() >= revdebounce)
                {
                    intake.setPower(-1);
                    hold.setPosition(holdopen);
                }

                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot15;
                }
                break;


            case shoot15:

                if (statetimer.seconds() >= 0.4)
                {
                    endshoot();
                    sotmActive = false;
                    intake.setPower(0);
                    follower.followPath(paths.Path16);
                    state = AutonState.followPath16;
                }
                break;



            case followPath16:
                if (!follower.isBusy())
                    state = AutonState.done;
                break;


            case done:
                intake.setPower(0);
                fly1.setPower(0);
                fly2.setPower(0);
                turret.setPower(0);
                break;
        }


        telemetry.addData("state",      state);
        telemetry.addData("ballstate",  bstate);
        telemetry.addData("sotm",       sotmActive);
        telemetry.addData("sensor v",   String.format("%.3f", d0.getVoltage()));
        telemetry.addData("fly v",      String.format("%.0f", fly2.getVelocity()));
        telemetry.addData("target v",   String.format("%.0f", targetvelocity));
        telemetry.addData("turret deg", String.format("%.1f", getturretdeg()));
        telemetry.update();
    }





    private void runballdetection()
    {
        if (d0.getVoltage() < dthresh)  { latch0 = true; st0.reset(); }
        if (d1.getVoltage() < dthresh1) { latch1 = true; st1.reset(); }
        if (d2.getVoltage() < dthresh2) { latch2 = true; st2.reset(); }
        if (st0.seconds() > sticky) latch0 = false;
        if (st1.seconds() > sticky) latch1 = false;
        if (st2.seconds() > sticky) latch2 = false;


        boolean hit = latch0 && latch1 && latch2;


        switch (bstate)
        {
            case idle:
                if (hit && !shooting)
                {
                    if (!prevdetect) { detecttimer.reset(); prevdetect = true; }
                    if (detecttimer.seconds() >= 0.2)
                    {
                        latch0 = latch1 = latch2 = false;
                        lpush.setPosition(lockpos);
                        rpush.setPosition(lockpos - servooff);
                        intake.setPower(-1);
                        revtimer.reset();
                        bstate = BallState.reversing;
                        prevdetect = false;
                        blinkin.setPosition(0.722);
                    }
                }
                else if (!hit)
                {
                    prevdetect = false;
                    blinkin.setPosition(0);
                }
                break;


            case reversing:
                hold.setPosition(holdclose);
                if (!shooting && revtimer.seconds() < 3.0)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                    intake.setPower(-1);
                }
                if (revtimer.seconds() >= 3.0)
                {
                    intake.setPower(1);
                    lpush.setPosition(push0);
                    rpush.setPosition(push0 - servooff);
                    bstate = BallState.locked;
                }
                break;


            case locked:
                lpush.setPosition(lockpos);
                rpush.setPosition(lockpos - servooff);
                if (!shooting) intake.setPower(1);
                break;
        }
    }





    private void startshoot()
    {
        lpush.setPosition(push3);
        rpush.setPosition(push3 - servooff);
        shooting = true;
        dipping  = false;
        dipdone  = false;
    }


    private void endshoot()
    {
        lpush.setPosition(push0);
        rpush.setPosition(push0 - servooff);
        hold.setPosition(holdclose);
        shooting   = false;
        bstate     = BallState.idle;
        prevdetect = false;
    }





    private void regressions()
    {
        double hoodpos, vel;
        switch (state)
        {
            case followPath1: case shoot1:
            hoodpos = hood1;  vel = vel1;  break;
            case followPath2:
            case followPath3: case shoot3:
            hoodpos = hood3;  vel = vel3;  break;
            case followPath4: case followPath5: case shoot5:
            hoodpos = hood5;  vel = vel5;  break;
            case followPath7: case shoot7:
            hoodpos = hood7;  vel = vel7;  break;
            case followPath9: case shoot9:
            hoodpos = hood9;  vel = vel9;  break;
            case followPath11: case shoot11:
            hoodpos = hood11; vel = vel11; break;
            case followPath13: case shoot13:
            hoodpos = hood13; vel = vel13; break;

            case followPath15: case shoot15:
            hoodpos = hood15; vel = vel15; break;
            case followPath16:
                hoodpos = hoodrest; vel = 0;   break;
            default:
                hoodpos = hoodrest; vel = velrest; break;
        }
        hoodbase = Math.max(0.0, Math.min(1.0, hoodpos));
        double maxvel = fly2.getMotorType().getMaxRPM() * fly2.getMotorType().getTicksPerRev() / 60.0;
        targetvelocity = Math.max(-maxvel, Math.min(maxvel, vel));
    }





    private void dipshot()
    {
        if (shooting && !dipping && !dipdone) { dipping = true; diptimer.reset(); }
        if (!shooting)
        {
            dipping = false;
            dipdone = false;
            hood.setPosition(hoodbase);
            return;
        }
        if (dipping)
        {
            double t = diptimer.seconds();
            if (t < dipdelay)
                hood.setPosition(hoodbase);
            else if (t < dipdelay + dipdur)
                hood.setPosition(Math.max(0.0, hoodbase - dipamt));
            else
            {
                hood.setPosition(hoodbase);
                dipping = false;
                dipdone = true;
            }
        }
    }





    private Pose virtualgoal(Pose p)
    {
        Vector vel = follower.getVelocity();
        double vx = vel != null ? vel.getXComponent() : 0.0;
        double vy = vel != null ? vel.getYComponent() : 0.0;
        if (Math.hypot(vx, vy) < 1.5) { vx = 0; vy = 0; }
        double dist     = Math.hypot(goalx - p.getX(), goaly - p.getY());
        double shottime = timeA * dist * dist + timeB * dist + timeC;
        return new Pose(goalx - vx * shottime, goaly - vy * shottime, 0);
    }





    private void aim()
    {
        double tgtangle = 0;
        double clamped  = clampturret();
        if (Math.abs(clamped - getturretdeg()) <= thresh) tgtangle = clamped;



        if (sotmActive) tgtangle += sotmBias;


        double cur = getturretdeg();
        double err = wrapangle(tgtangle - cur);


        if (Math.abs(err) <= tdead) { turret.setPower(0); return; }


        tpid.setPID(tkp, tki, tkd);
        double out   = tpid.calculate(cur, tgtangle);
        double ff    = Math.abs(err) > tffdead ? Math.copySign(tks, err) : 0.0;
        double power = Math.max(-tmax, Math.min(tmax, out + ff));
        turret.setPower(power);
    }





    private void runflywheel()
    {
        if (Math.abs(targetvelocity) <= 1.0)
        {
            fly1.setPower(0);
            fly2.setPower(0);
            return;
        }
        double maxvel = fly2.getMotorType().getMaxRPM() * fly2.getMotorType().getTicksPerRev() / 60.0;
        double batv   = Math.max(10.5, vsensor.getVoltage());
        double ff     = bangff * (targetvelocity / maxvel) * (12.0 / batv);
        double bb     = fly2.getVelocity() < targetvelocity ? 1.0 : 0.0;
        double power  = Math.min(1.0, bb + ff);
        fly1.setPower(power);
        fly2.setPower(power);
    }



    // Symmetric wrap: turret encoder reading is normalized into (-180, 180].
    private double getturretdeg()
    {
        double angle = (tenc.getVoltage() / tenc.getMaxVoltage()) * 360.0 - tzero;
        return wrapangle(angle);
    }



    // Simple clamp on the raw target degree value — no re-wrap needed since
    // every turret target constant (turret1..turret15, turretrest) already
    // lives well inside the symmetric ±thresh range.
    private double clampturret()
    {
        double rawangle;
        switch (state)
        {
            case followPath1: case shoot1:   rawangle = turret1;    break;
            case followPath2:                rawangle = 65;         break;
            case followPath3: case shoot3:   rawangle = turret3;    break;
            case followPath5: case shoot5:   rawangle = turret5;    break;
            case followPath7: case shoot7:   rawangle = turret7;    break;
            case followPath9: case shoot9:   rawangle = turret9;    break;
            case followPath11: case shoot11: rawangle = turret11;   break;
            case followPath13: case shoot13: rawangle = turret13;   break;
            case followPath15: case shoot15: rawangle = turret15;   break;
            default:                         rawangle = turretrest; break;
        }
        if (rawangle >  thresh2) return  thresh2;
        if (rawangle < -thresh)  return -thresh;
        return rawangle;
    }


    private double wrapangle(double a) { return ((a + 180) % 360 + 360) % 360 - 180; }


    @Override public void opLoopHook() {}


    @Override
    public void opTeardown()
    {
        org.firstinspires.ftc.teamcode.tele.PoseCache.lastPose = follower.getPose();
    }
}