package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
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
@Autonomous(name = "fourgatered")
public class fourgatered extends DbzOpMode
{
    // --- SERVO / INTAKE TUNING ---
    public static double servooff = 0.01;
    public static double push0 = 0.81, push3 = 0.22;
    public static double lockpos = 0.71;
    public static double holdopen = 0.8, holdclose = 0.467;
    public static double intakewaittimeout = 1.5;
    public static double revdebounce = 1.7, lockdebounce = 1.4;

    // --- FLYWHEEL / HOOD ---
    public static double bangff = 0.85;
    public static double hooddefault = 0.5;

    // --- SHOT TIMING ---
    public static double timeA = 0.00002, timeB = 0.004, timeC = 0.25;
    public static double dipamt = 0, dipdelay = 0.5, dipdur = 0.15;

    // --- GOAL / FIELD ---
    public static double goalx = 144.5, goaly = 140;
    public static double startx = 127, starty = 108.37515052508749;
    public static double tvalue = .53;
    //gate
    public static double gatex = 131.9, gatey = 53.8, gateh = 22.35;
    //return
    public static double returnposx=88, returnposy=68;
    public static double nearwallx = 126.373, nearwally = 84.566;

    // --- TURRET PID ---
    public static double tzero = 181;
    public static double tkp = 0.02, tki = 0.0, tkd = 0.001;
    public static double tdead = 0.0, tmax = 1.0, tks = 0.0, tffdead = 0.0;
    public static double thresh = 220, thresh2 = 180;

    // --- DISTANCE SENSOR THRESHOLDS ---
    public static double dthresh = 0.157, dthresh1 = 0.173, dthresh2 = 0.155;
    public static double sticky = 0.15;

    // --- PER-SHOT TUNING (hood pos, flywheel velocity, turret angle) ---
    public static double s1_hood  = 0.17,  s1_vel  = 1200, s1_turret  = -20;
    public static double s3_hood  = 0.17,  s3_vel  = 1200, s3_turret  = -20;
    public static double s5_hood  = 0.499,  s5_vel  = 1600, s5_turret  = -121;
    public static double s7_hood  = 0.499,  s7_vel  = 1600, s7_turret  = -121;
    public static double s9_hood  = 0.499,  s9_vel  = 1600, s9_turret  = -121;
    public static double s11_hood = 0.499,  s11_vel = 1600, s11_turret = -121;
    public static double s13_hood = 0.499,  s13_vel = 1600, s13_turret = -121;
    public static double s15_hood = 0.499,  s15_vel = 1600, s15_turret = -121;
    public static double s17_hood = 0.499,  s17_vel = 1600, s17_turret = -121;

    // -------------------------------------------------------------------------
    //  PATHS
    // -------------------------------------------------------------------------
    public static class Paths
    {
        public PathChain Path1,  Path2,  Path3,  Path4,  Path5,  Path6,
                Path7,  Path8,  Path9,  Path10, Path11, Path12,
                Path13, Path14, Path15, Path16, Path17, Path18;

        public Paths(Follower f)
        {
            // preload shoot position
            Path1 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(126.460, 108.375), new Pose(118.9900, 97.877)))
                    .setConstantHeadingInterpolation(Math.toRadians(270)).build();

            // intake s1
            Path2 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(118.9900, 97.877), new Pose(119.5900, 79.171)))
                    .setTangentHeadingInterpolation().build();

            // return from s1 intake → shoot position
            Path3 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(119.5900, 79.171), new Pose(118.9900, 97.877)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // intake s2
            Path4 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(118.9900, 97.877), new Pose(118.985, 57.369)))
                    .setTangentHeadingInterpolation().build();

            // gate clear
            Path5 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(118.985, 57.369), new Pose(126.94, 68.758)))
                    .setConstantHeadingInterpolation(Math.toRadians(270)).build();

            // prep gate cycle pos
            Path6 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(126.94, 68.758), new Pose(returnposx, returnposy)))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, .1, HeadingInterpolator.constant(Math.toRadians(270))),
                                    new HeadingInterpolator.PiecewiseNode(.1, 1, HeadingInterpolator.tangent.reverse())
                            )
                    ).build();

            // --- GATE CYCLE 1: approach gate ---
            Path7 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(returnposx, returnposy),
                                    new Pose(gatex, gatey)
                            ))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, tvalue, HeadingInterpolator.tangent),
                                    new HeadingInterpolator.PiecewiseNode(tvalue, 1, HeadingInterpolator.constant(Math.toRadians(gateh)))
                            )
                    ).build();

            // GATE CYCLE 1: return to shoot pos
            Path8 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(130, 60.987), new Pose(returnposx, returnposy)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // --- GATE CYCLE 2: approach gate ---
            Path9 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(returnposx, returnposy),
                                    new Pose(gatex, gatey)
                            ))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, tvalue, HeadingInterpolator.tangent),
                                    new HeadingInterpolator.PiecewiseNode(tvalue, 1, HeadingInterpolator.constant(Math.toRadians(gateh)))
                            )
                    ).build();

            // GATE CYCLE 2: return to shoot pos
            Path10 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(135.990, 60.987), new Pose(returnposx, returnposy)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // --- GATE CYCLE 3: approach gate ---
            Path11 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(returnposx, returnposy),
                                    new Pose(gatex, gatey)
                            ))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, tvalue, HeadingInterpolator.tangent),
                                    new HeadingInterpolator.PiecewiseNode(tvalue, 1, HeadingInterpolator.constant(Math.toRadians(gateh)))
                            )
                    ).build();

            // GATE CYCLE 3: return to shoot pos
            Path12 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(135.990, 60.987), new Pose(returnposx, returnposy)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // --- GATE CYCLE 4: approach gate ---
            Path13 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(returnposx, returnposy),
                                    new Pose(gatex, gatey)
                            ))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, tvalue, HeadingInterpolator.tangent),
                                    new HeadingInterpolator.PiecewiseNode(tvalue, 1, HeadingInterpolator.constant(Math.toRadians(gateh)))
                            )
                    ).build();

            // GATE CYCLE 4: return to shoot pos
            Path14 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(135.990, 60.987), new Pose(returnposx, returnposy)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // --- GATE CYCLE 5: approach gate ---
            Path15 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(returnposx, returnposy),
                                    new Pose(gatex, gatey)
                            ))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, tvalue, HeadingInterpolator.tangent),
                                    new HeadingInterpolator.PiecewiseNode(tvalue, 1, HeadingInterpolator.constant(Math.toRadians(gateh)))
                            )
                    ).build();

            // GATE CYCLE 5: return to shoot pos
            Path16 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(135.990, 60.987), new Pose(returnposx, returnposy)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // --- GATE CYCLE 6: approach gate ---
            Path17 = f.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(returnposx, returnposy),
                                    new Pose(gatex, gatey)
                            ))
                    .setHeadingInterpolation(
                            HeadingInterpolator.piecewise(
                                    new HeadingInterpolator.PiecewiseNode(0, tvalue, HeadingInterpolator.tangent),
                                    new HeadingInterpolator.PiecewiseNode(tvalue, 1, HeadingInterpolator.constant(Math.toRadians(gateh)))
                            )
                    ).build();

            // GATE CYCLE 6: return to final shoot pos
            Path18 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(135.990, 60.987), new Pose(returnposx, returnposy)))
                    .setTangentHeadingInterpolation().setReversed().build();
        }
    }

    // -------------------------------------------------------------------------
    //  HARDWARE
    // -------------------------------------------------------------------------
    protected Servo rpush, lpush, hood, hold, blinkin;
    protected DcMotorEx intake, fly1, fly2, turret;
    private VoltageSensor vsensor;
    private AnalogInput tenc, d0, d1, d2;
    private PIDController tpid;
    private Follower follower;
    private Paths paths;

    // -------------------------------------------------------------------------
    //  STATE MACHINES
    // -------------------------------------------------------------------------
    private enum AutonState
    {
        // Preload
        followPath1, shoot1,
        // First spike intake → return → shoot
        followPath2, followPath3, shoot3,
        // Second spike intake → gate clear → shoot
        followPath4, followPath5, followPath6, shoot6,
        // Gate cycle 1: approach → intakeWait → return → shoot
        followPath7,  intakeWait1,
        followPath8,  shoot8,
        // Gate cycle 2
        followPath9,  intakeWait2,
        followPath10, shoot10,
        // Gate cycle 3
        followPath11, intakeWait3,
        followPath12, shoot12,
        // Gate cycle 4
        followPath13, intakeWait4,
        followPath14, shoot14,
        // Gate cycle 5
        followPath15, intakeWait5,
        followPath16, shoot16,
        // Gate cycle 6
        followPath17, intakeWait6,
        followPath18, shoot18,
        done
    }
    private AutonState state = AutonState.followPath1;

    private enum BallState { idle, reversing, locked }
    private BallState bstate = BallState.idle;

    // -------------------------------------------------------------------------
    //  TIMERS / FLAGS
    // -------------------------------------------------------------------------
    private ElapsedTime statetimer  = new ElapsedTime();
    private ElapsedTime revtimer    = new ElapsedTime();
    private ElapsedTime detecttimer = new ElapsedTime();
    private ElapsedTime diptimer    = new ElapsedTime();
    private ElapsedTime st0 = new ElapsedTime();
    private ElapsedTime st1 = new ElapsedTime();
    private ElapsedTime st2 = new ElapsedTime();
    private boolean latch0 = false, latch1 = false, latch2 = false;

    private boolean prevdetect    = false;
    private double  targetvelocity = 0;
    private double  hoodbase      = hooddefault;
    private boolean shooting      = false;
    private boolean dipping = false, dipdone = false;

    // -------------------------------------------------------------------------
    //  INIT
    // -------------------------------------------------------------------------
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

        hood.setPosition(hooddefault);
        hold.setPosition(holdopen);
        lpush.setPosition(lockpos);
        rpush.setPosition(lockpos - servooff);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startx, starty, Math.toRadians(270)));
        paths = new Paths(follower);

        follower.followPath(paths.Path1, true);
        statetimer.reset();
    }

    // -------------------------------------------------------------------------
    //  MAIN LOOP
    // -------------------------------------------------------------------------
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
            // ------------------------------------------------------------------
            // PRELOAD: drive to shoot pos, shoot
            // ------------------------------------------------------------------
            case followPath1:
                hold.setPosition(holdopen);
                if (follower.getCurrentTValue() > 0.9)
                {
                    intake.setPower(1);
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot1;
                }
                break;

            case shoot1:
                if (statetimer.seconds() >= 0.45)
                {
                    endshoot();
                    follower.followPath(paths.Path2);
                    statetimer.reset();
                    state = AutonState.followPath2;
                }
                break;

            // ------------------------------------------------------------------
            // FIRST SPIKE: intake on Path2, return on Path3, shoot
            // ------------------------------------------------------------------
            case followPath2:
                hold.setPosition(holdclose);
                intake.setPower(1);
                runballdetection();
                if (!follower.isBusy())
                {
                    follower.followPath(paths.Path3);
                    statetimer.reset();
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
                if (statetimer.seconds() >= 0.45)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path4);
                    statetimer.reset();
                    state = AutonState.followPath4;
                }
                break;

            // ------------------------------------------------------------------
            // SECOND SPIKE: intake on Path4, gate clear on Path5,
            // move to shoot pos on Path6, shoot
            // ------------------------------------------------------------------
            case followPath4:
                intake.setPower(1);
                runballdetection();
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    follower.followPath(paths.Path5, true);
                    statetimer.reset();
                    state = AutonState.followPath5;
                }
                break;

            case followPath5:
                hold.setPosition(holdclose);

                // gate clear move - no intake, hold locked
                if (statetimer.seconds() > 0.4)  { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= 0.6)  intake.setPower(-1);
                if (statetimer.seconds() > 1)     hold.setPosition(holdopen);
                if (!follower.isBusy())
                {
                    follower.followPath(paths.Path6, true);
                    statetimer.reset();
                    state = AutonState.followPath6;
                }
                break;

            case followPath6:
                hold.setPosition(holdopen);

                intake.setPower(1);
                runballdetection();
                hold.setPosition(holdclose);
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot6;
                }
                break;

            case shoot6:
                if (statetimer.seconds() >= 0.37)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path7);
                    statetimer.reset();
                    state = AutonState.followPath7;
                }
                break;

            // ------------------------------------------------------------------
            // GATE CYCLE 1: approach gate (Path7) → intakeWait → return (Path8) → shoot
            // ------------------------------------------------------------------
            case followPath7:
                hold.setPosition(holdclose);

                lpush.setPosition(push0);
                rpush.setPosition(push0 - servooff);
                intake.setPower(1);
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait1;
                }
                break;

            case intakeWait1:
                runballdetection();
                hold.setPosition(holdclose);
                if (statetimer.seconds() > 1.6)  { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= 1.65) intake.setPower(-1);
                if (statetimer.seconds() > 1)     hold.setPosition(holdopen);
                if (bstate == BallState.locked || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path8);
                    statetimer.reset();
                    state = AutonState.followPath8;
                }
                break;

            case followPath8:
                hold.setPosition(holdopen);

                intake.setPower(1);
                hold.setPosition(holdopen);
                runballdetection();
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot8;
                }
                break;

            case shoot8:
                if (statetimer.seconds() >= 0.37)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path9);
                    statetimer.reset();
                    state = AutonState.followPath9;
                }
                break;

            // ------------------------------------------------------------------
            // GATE CYCLE 2
            // ------------------------------------------------------------------
            case followPath9:
                hold.setPosition(holdclose);

                lpush.setPosition(push0);
                rpush.setPosition(push0 - servooff);
                intake.setPower(1);
                hold.setPosition(holdopen);
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait1;
                }
                break;

            case intakeWait2:
                runballdetection();
                hold.setPosition(holdclose);

                if (statetimer.seconds() > 1.6)  { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= 1.65) intake.setPower(-1);
                if (statetimer.seconds() > 1)     hold.setPosition(holdopen);
                if (bstate == BallState.locked || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path10);
                    statetimer.reset();
                    state = AutonState.followPath10;
                }
                break;

            case followPath10:
                hold.setPosition(holdopen);

                intake.setPower(1);
                hold.setPosition(holdopen);
                runballdetection();
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot10;
                }
                break;

            case shoot10:
                if (statetimer.seconds() >= 0.37)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path11);
                    statetimer.reset();
                    state = AutonState.followPath11;
                }
                break;

            // ------------------------------------------------------------------
            // GATE CYCLE 3
            // ------------------------------------------------------------------
            case followPath11:
                hold.setPosition(holdclose);

                lpush.setPosition(push0);
                rpush.setPosition(push0 - servooff);
                intake.setPower(1);
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait1;
                }
                break;

            case intakeWait3:
                runballdetection();
                hold.setPosition(holdclose);

                if (statetimer.seconds() > 1.6)  { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= 1.65) intake.setPower(-1);
                if (statetimer.seconds() > 1)     hold.setPosition(holdopen);
                if (bstate == BallState.locked || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path12);
                    statetimer.reset();
                    state = AutonState.followPath12;
                }
                break;

            case followPath12:
                hold.setPosition(holdopen);

                intake.setPower(1);
                runballdetection();
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot12;
                }
                break;

            case shoot12:
                if (statetimer.seconds() >= .37)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path13);
                    statetimer.reset();
                    state = AutonState.followPath13;
                }
                break;

            // ------------------------------------------------------------------
            // GATE CYCLE 4
            // ------------------------------------------------------------------
            case followPath13:
                lpush.setPosition(push0);
                rpush.setPosition(push0 - servooff);
                intake.setPower(1);
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait1;
                }
                break;

            case intakeWait4:
                runballdetection();
                hold.setPosition(holdclose);

                if (statetimer.seconds() > 1.6)  { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= 1.65) intake.setPower(-1);
                if (statetimer.seconds() > 1)     hold.setPosition(holdopen);
                if (bstate == BallState.locked || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path14);
                    statetimer.reset();
                    state = AutonState.followPath14;
                }
                break;

            case followPath14:
                intake.setPower(1);
                hold.setPosition(holdopen);
                runballdetection();
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot14;
                }
                break;

            case shoot14:
                if (statetimer.seconds() >= 0.37)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path15);
                    statetimer.reset();
                    state = AutonState.followPath15;
                }
                break;

            // ------------------------------------------------------------------
            // GATE CYCLE 5
            // ------------------------------------------------------------------
            case followPath15:
                lpush.setPosition(push0);
                rpush.setPosition(push0 - servooff);
                intake.setPower(1);
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait1;
                }
                break;

            case intakeWait5:
                runballdetection();
                hold.setPosition(holdclose);

                if (statetimer.seconds() > 1.6)  { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= 1.65) intake.setPower(-1);
                if (statetimer.seconds() > 1)     hold.setPosition(holdopen);
                if (bstate == BallState.locked || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path16);
                    statetimer.reset();
                    state = AutonState.followPath16;
                }
                break;

            case followPath16:
                intake.setPower(1);
                hold.setPosition(holdopen);
                runballdetection();
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot16;
                }
                break;

            case shoot16:
                if (statetimer.seconds() >= 0.37)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path17);
                    statetimer.reset();
                    state = AutonState.followPath17;
                }
                break;

            // ------------------------------------------------------------------
            // GATE CYCLE 6 (final)
            // ------------------------------------------------------------------
            case followPath17:
                lpush.setPosition(push0);
                rpush.setPosition(push0 - servooff);
                intake.setPower(1);
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait1;
                }
                break;

            case intakeWait6:
                runballdetection();
                hold.setPosition(holdclose);

                if (statetimer.seconds() > 1.6)  { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= 1.65) intake.setPower(-1);
                if (statetimer.seconds() > 1)     hold.setPosition(holdopen);
                if (bstate == BallState.locked || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path18, true);
                    statetimer.reset();
                    state = AutonState.followPath18;
                }
                break;

            case followPath18:
                intake.setPower(1);
                hold.setPosition(holdopen);
                runballdetection();
                if (follower.getCurrentTValue() > 0.95)
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot18;
                }
                break;

            case shoot18:
                if (statetimer.seconds() >= 0.37)
                {
                    endshoot();
                    state = AutonState.done;
                }
                break;

            // ------------------------------------------------------------------
            case done:
                intake.setPower(0);
                fly1.setPower(0);
                fly2.setPower(0);
                turret.setPower(0);
                break;
        }

        telemetry.addData("state",      state);
        telemetry.addData("ballstate",  bstate);
        telemetry.addData("sensor v",   String.format("%.3f", d0.getVoltage()));
        telemetry.addData("fly v",      String.format("%.0f", fly2.getVelocity()));
        telemetry.addData("target v",   String.format("%.0f", targetvelocity));
        telemetry.addData("turret deg", String.format("%.1f", getturretdeg()));
        telemetry.update();
    }

    // -------------------------------------------------------------------------
    //  BALL DETECTION
    // -------------------------------------------------------------------------
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
                hold.setPosition(holdopen);
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

    // -------------------------------------------------------------------------
    //  SHOOT HELPERS
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    //  PER-SHOT TUNING  (hood + velocity, keyed by state)
    // -------------------------------------------------------------------------
    private void regressions()
    {
        double hoodpos, vel;

        if      (state == AutonState.shoot1  || state == AutonState.followPath1)
        { hoodpos = s1_hood;  vel = s1_vel; }
        else if (state == AutonState.shoot3  || state == AutonState.followPath3)
        { hoodpos = s3_hood;  vel = s3_vel; }
        else if (state == AutonState.shoot6  || state == AutonState.followPath6)
        { hoodpos = s5_hood;  vel = s5_vel; }
        else if (state == AutonState.shoot8  || state == AutonState.followPath8)
        { hoodpos = s7_hood;  vel = s7_vel; }
        else if (state == AutonState.shoot10 || state == AutonState.followPath10)
        { hoodpos = s9_hood;  vel = s9_vel; }
        else if (state == AutonState.shoot12 || state == AutonState.followPath12)
        { hoodpos = s11_hood; vel = s11_vel; }
        else if (state == AutonState.shoot14 || state == AutonState.followPath14)
        { hoodpos = s13_hood; vel = s13_vel; }
        else if (state == AutonState.shoot16 || state == AutonState.followPath16)
        { hoodpos = s15_hood; vel = s15_vel; }
        else if (state == AutonState.shoot18 || state == AutonState.followPath18)
        { hoodpos = s17_hood; vel = s17_vel; }
        else
        { hoodpos = hooddefault; vel = 0; }

        hoodbase = Math.max(0.0, Math.min(1.0, hoodpos));
        double maxvel = fly2.getMotorType().getMaxRPM() * fly2.getMotorType().getTicksPerRev() / 60.0;
        targetvelocity = Math.max(-maxvel, Math.min(maxvel, vel));
    }

    // -------------------------------------------------------------------------
    //  DIP SHOT
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    //  VIRTUAL GOAL (velocity compensation)
    // -------------------------------------------------------------------------
    private Pose virtualgoal(Pose p)
    {
        Vector vel = follower.getVelocity();
        double vx  = vel != null ? vel.getXComponent() : 0.0;
        double vy  = vel != null ? vel.getYComponent() : 0.0;
        if (Math.hypot(vx, vy) < 1.5) { vx = 0; vy = 0; }
        double dist     = Math.hypot(goalx - p.getX(), goaly - p.getY());
        double shottime = timeA * dist * dist + timeB * dist + timeC;
        return new Pose(goalx - vx * shottime, goaly - vy * shottime, 0);
    }

    // -------------------------------------------------------------------------
    //  TURRET AIM
    // -------------------------------------------------------------------------
    private void aim()
    {
        double tgtangle = 0;
        double clamped  = clampturret();
        if (Math.abs(clamped - getturretdeg()) <= thresh) tgtangle = clamped;

        double cur = getturretdeg();
        double err = wrapangle(tgtangle - cur);

        if (Math.abs(err) <= tdead) { turret.setPower(0); return; }

        tpid.setPID(tkp, tki, tkd);
        double out   = tpid.calculate(cur, tgtangle);
        double ff    = Math.abs(err) > tffdead ? Math.copySign(tks, err) : 0.0;
        double power = out + ff;
        power = Math.max(-tmax, Math.min(tmax, power));
        turret.setPower(power);
    }

    // -------------------------------------------------------------------------
    //  PER-SHOT TURRET TARGET  (keyed by state)
    // -------------------------------------------------------------------------
    private double clampturret()
    {
        double rawangle;

        if      (state == AutonState.shoot1  || state == AutonState.followPath1)
            rawangle = s1_turret;
        else if (state == AutonState.shoot3  || state == AutonState.followPath3)
            rawangle = s3_turret;
        else if (state == AutonState.shoot6  || state == AutonState.followPath6)
            rawangle = s5_turret;
        else if (state == AutonState.shoot8  || state == AutonState.followPath8)
            rawangle = s7_turret;
        else if (state == AutonState.shoot10 || state == AutonState.followPath10)
            rawangle = s9_turret;
        else if (state == AutonState.shoot12 || state == AutonState.followPath12)
            rawangle = s11_turret;
        else if (state == AutonState.shoot14 || state == AutonState.followPath14)
            rawangle = s13_turret;
        else if (state == AutonState.shoot16 || state == AutonState.followPath16)
            rawangle = s15_turret;
        else if (state == AutonState.shoot18 || state == AutonState.followPath18)
            rawangle = s17_turret;
        else
            rawangle = 51;

        double d = wrapasym(rawangle, thresh);
        if (d >  thresh2) return  thresh2;
        if (d < -thresh)  return -thresh;
        return d;
    }

    // -------------------------------------------------------------------------
    //  FLYWHEEL (bang-bang + feedforward)
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    //  TURRET ENCODER HELPERS
    // -------------------------------------------------------------------------
    private double getturretdeg()
    {
        double angle = (tenc.getVoltage() / tenc.getMaxVoltage()) * 360.0 - tzero;
        return wrapasym(angle, thresh);
    }

    private double wrapangle(double a) { return ((a + 180) % 360 + 360) % 360 - 180; }
    private double wrapasym(double a, double n) { return ((a + n) % 360 + 360) % 360 - n; }

    @Override public void opLoopHook() {}

    @Override
    public void opTeardown()
    {
        org.firstinspires.ftc.teamcode.tele.PoseCache.lastPose = follower.getPose();
    }
}