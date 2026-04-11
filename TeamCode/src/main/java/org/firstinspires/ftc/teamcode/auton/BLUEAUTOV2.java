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
@Autonomous(name = "BLUEAUTOV2")
public class BLUEAUTOV2 extends DbzOpMode
{
    public static double servooff = 0.01;
    public static double push0 = 0.81, push3 = 0.22;
    public static double lockpos = 0.71;
    public static double holdopen = 0.8, holdclose = 0.467;
    public static double intakewaittimeout = 1;
    public static double bangff = 0.85;
    public static double hooddefault = 0.5;
    public static double timeA = 0.00002, timeB = 0.004, timeC = 0.25;
    public static double goalx = 0, goaly = 144;
    public static double dipamt = 0, dipdelay = 0.5, dipdur = 0.15;
    public static double tzero = 187;
    public static double tkp = 0.02, tki = 0.0, tkd = 0.001;
    public static double tdead = 0.0, tmax = 1.0, tks = 0.0, tffdead = 0.0;
    public static double thresh = 220, thresh2 = 180;
    public static double startx = 144 - 114.2417, starty = 133.472;
    public static double gatex = 144 - 145.56, gatey = 61.28, gateh = 21;
    public static double dthresh = 0.157, dthresh1 = 0.173, dthresh2 = 0.155;
    public static double sticky = 0.15;
    public static double revdebounce = 1.3, lockdebounce = 1.1;


    public static class Paths
    {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7,
                Path8, Path9, Path10, Path11, Path12, Path13;


        public Paths(Follower f)
        {
            Path1 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(144-111.417, 136.815), new Pose(144-98.149, 83.168)))
                    .setTangentHeadingInterpolation().build();


            Path2 = f.pathBuilder().addPath(
                            new BezierCurve(new Pose(144-98.149, 83.168), new Pose(144-110.51829161451816, 62.02878598247808), new Pose(144-130.64287359199, 58.48279098873591)))
                    .setTangentHeadingInterpolation().build();


            Path3 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(144-130.64287359199, 58.48279098873591), new Pose(144-97.149, 77.168)))
                    .setTangentHeadingInterpolation().setReversed().build();


            Path4 = f.pathBuilder().addPath(
                            new BezierCurve(new Pose(144-97.149, 77.168), new Pose(144-110.990, 60.361), new Pose(gatex, gatey)))
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians
                            (180-gateh)).build();


            Path5 = f.pathBuilder().addPath(
                            new BezierCurve(new Pose(gatex, gatey), new Pose(144-110.990, 60.361), new Pose(144-97.149, 77.168)))
                    .setLinearHeadingInterpolation(Math.toRadians(180-gateh), Math.toRadians(180-0)).build();


            Path6 = f.pathBuilder().addPath(
                            new BezierCurve(new Pose(144-97.149, 77.168), new Pose(144-110.990, 60.361), new Pose(gatex, gatey)))
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-gateh)).build();


            Path7 = f.pathBuilder().addPath(
                            new BezierCurve(new Pose(gatex, gatey), new Pose(144-110.990, 60.361), new Pose(144-97.149, 77.168)))
                    .setLinearHeadingInterpolation(Math.toRadians(180-gateh), Math.toRadians(180-0)).build();


            Path8 = f.pathBuilder().addPath(
                            new BezierCurve(new Pose(144-97.149, 77.168), new Pose(144-110.990, 60.361), new Pose(gatex, gatey)))
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Math.toRadians(180-gateh)).build();


            Path9 = f.pathBuilder().addPath(
                            new BezierCurve(new Pose(gatex, gatey), new Pose(144-110.990, 60.361), new Pose(144-97.149, 84.566)))
                    .setLinearHeadingInterpolation(Math.toRadians(180-gateh), Math.toRadians(180-0)).build();


            Path10 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(144-97.149, 84.566), new Pose(144-128.573, 84.566)))
                    .setTangentHeadingInterpolation().build();


            Path11 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(144-128.573, 84.566), new Pose(144-95.149, 84.168)))
                    .setTangentHeadingInterpolation().setReversed().build();


            Path12 = f.pathBuilder().addPath(
                            new BezierCurve(new Pose(144-85.149, 84.168), new Pose(144-110.25156445556946, 40.73091364205258), new Pose(144-138.54693366708383, 28.59198998748437)))
                    .setTangentHeadingInterpolation().build();


            Path13 = f.pathBuilder().addPath(
                            new BezierLine(new Pose(144-138.54693366708383, 28.59198998748437), new Pose(144-96.5, 112)))
                    .setTangentHeadingInterpolation().setReversed().build();
        }
    }


    protected Servo rpush, lpush, hood, hold, blinkin;
    protected DcMotorEx intake, fly1, fly2, turret;
    private VoltageSensor vsensor;
    private AnalogInput tenc, d0, d1, d2;
    private PIDController tpid;
    private Follower follower;
    private Paths paths;


    private enum AutonState
    {
        followPath1, shoot1,
        followPath2,
        followPath3, shoot3,
        followPath4, intakeWait1,
        followPath5, shoot5,
        followPath6, intakeWait2,
        followPath7, shoot7,
        followPath8, intakeWait3,
        followPath9, shoot9,
        followPath10,
        followPath11, shoot11,
        followPath12,
        followPath13, shoot13,
        done
    }
    private AutonState state = AutonState.followPath1;


    private enum BallState { idle, reversing, locked }
    private BallState bstate = BallState.idle;


    private ElapsedTime statetimer = new ElapsedTime();
    private ElapsedTime revtimer = new ElapsedTime();
    private ElapsedTime detecttimer = new ElapsedTime();
    private ElapsedTime diptimer = new ElapsedTime();
    private ElapsedTime st0 = new ElapsedTime();
    private ElapsedTime st1 = new ElapsedTime();
    private ElapsedTime st2 = new ElapsedTime();
    private boolean latch0 = false, latch1 = false, latch2 = false;


    private boolean prevdetect = false;
    private double targetvelocity = 0;
    private double hoodbase = hooddefault;
    private boolean shooting = false;
    private boolean dipping = false, dipdone = false;


    @Override
    public void opInit()
    {
        rpush = hardwareMap.get(Servo.class, "rightpushServo");
        lpush = hardwareMap.get(Servo.class, "leftpushServo");
        hood = hardwareMap.get(Servo.class, "hoodServo");
        hold = hardwareMap.get(Servo.class, "holdServo");
        blinkin = hardwareMap.get(Servo.class, "light");


        d0 = hardwareMap.get(AnalogInput.class, "distancez");
        d1 = hardwareMap.get(AnalogInput.class, "distance1");
        d2 = hardwareMap.get(AnalogInput.class, "distance2");
        tenc = hardwareMap.get(AnalogInput.class, "turretEncoder");


        intake = robot.intakeMotor;
        fly1 = robot.outtake1Motor;
        fly2 = robot.outtake2Motor;
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
                if (!follower.isBusy())
                {
                    intake.setPower(1);
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot1;
                }
                break;


            case shoot1:
                if (statetimer.seconds() >= 0.5)
                {
                    endshoot();
                    follower.followPath(paths.Path2, true);
                    state = AutonState.followPath2;
                }
                break;


            case followPath2:
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    follower.followPath(paths.Path3, true);
                    intake.setPower(1);
                    state = AutonState.followPath3;
                }
                break;


            case followPath3:
                hold.setPosition(holdopen);
                if (!follower.isBusy())
                {
                    intake.setPower(1);
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot3;
                }
                break;


            case shoot3:
                if (statetimer.seconds() >= 0.6)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path4, true);
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
                    state = AutonState.intakeWait1;
                }
                break;


            case intakeWait1:
                runballdetection();
                if (statetimer.seconds() >= 0.28)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path5, true);
                    state = AutonState.followPath5;
                }
                break;


            case followPath5:
                if (statetimer.seconds() > 0.5)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                }
                if (statetimer.seconds() >= 0.5) intake.setPower(-1);
                if (statetimer.seconds() > 0.8) hold.setPosition(holdopen);
                if (!follower.isBusy())
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot5;
                }
                break;


            case shoot5:
                if (statetimer.seconds() >= 0.5)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path6, true);
                    state = AutonState.followPath6;
                }
                break;


            case followPath6:
                intake.setPower(1);
                hold.setPosition(holdclose);
                runballdetection();
                if (!follower.isBusy())
                {
                    statetimer.reset();
                    state = AutonState.intakeWait2;
                }
                break;


            case intakeWait2:
                runballdetection();
                if (bstate == BallState.reversing || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path7, true);
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
                if (!follower.isBusy())
                {
                    startshoot();
                    intake.setPower(1);
                    statetimer.reset();
                    state = AutonState.shoot7;
                }
                break;


            case shoot7:
                if (statetimer.seconds() >= 0.5)
                {
                    endshoot();
                    intake.setPower(1);
                    follower.followPath(paths.Path8, true);
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
                if (bstate == BallState.reversing || statetimer.seconds() >= intakewaittimeout)
                {
                    bstate = BallState.idle;
                    prevdetect = false;
                    follower.followPath(paths.Path9, true);
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
                if (!follower.isBusy())
                {
                    startshoot();
                    intake.setPower(1);
                    statetimer.reset();
                    state = AutonState.shoot9;
                }
                break;


            case shoot9:
                runballdetection();
                if (statetimer.seconds() >= 0.5)
                {
                    endshoot();
                    follower.followPath(paths.Path10, true);
                    state = AutonState.followPath10;
                }
                break;


            case followPath10:
                intake.setPower(1);
                hold.setPosition(holdclose);
                if (!follower.isBusy())
                {
                    follower.followPath(paths.Path11, true);
                    statetimer.reset();
                    state = AutonState.followPath11;
                }
                break;


            case followPath11:
                hold.setPosition(holdopen);
                if (!follower.isBusy())
                {
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot11;
                }
                break;


            case shoot11:
                if (statetimer.seconds() >= 0.5)
                {
                    endshoot();
                    follower.followPath(paths.Path12, true);
                    hold.setPosition(holdclose);
                    state = AutonState.followPath12;
                }
                break;


            case followPath12:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.Path13, true);
                    statetimer.reset();
                    state = AutonState.followPath13;
                }
                break;


            case followPath13:
                lpush.setPosition(lockpos);
                rpush.setPosition(lockpos - servooff);
                if (statetimer.seconds() > 0.3)
                {
                    intake.setPower(-1);
                    hold.setPosition(holdopen);
                }
                if (!follower.isBusy())
                {
                    intake.setPower(1);
                    startshoot();
                    statetimer.reset();
                    state = AutonState.shoot13;
                }
                break;


            case shoot13:
                if (statetimer.seconds() >= 0.5)
                {
                    endshoot();
                    state = AutonState.done;
                }
                break;


            case done:
                intake.setPower(0);
                fly1.setPower(0);
                fly2.setPower(0);
                turret.setPower(0);
                break;
        }


        telemetry.addData("state", state);
        telemetry.addData("ballstate", bstate);
        telemetry.addData("sensor v", String.format("%.3f", d0.getVoltage()));
        telemetry.addData("fly v", String.format("%.0f", fly2.getVelocity()));
        telemetry.addData("target v", String.format("%.0f", targetvelocity));
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
                if (!shooting && revtimer.seconds() < 2.0)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                    intake.setPower(-1);
                }
                if (revtimer.seconds() >= 2.0)
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
        dipping = false;
        dipdone = false;
    }


    private void endshoot()
    {
        lpush.setPosition(push0);
        rpush.setPosition(push0 - servooff);
        shooting = false;
        bstate = BallState.idle;
        prevdetect = false;
    }


    private void regressions()
    {
        double vel;
        if (state == AutonState.shoot13 || state == AutonState.followPath13)
        {
            hoodbase = 0.22;
            vel = 1350;
        }
        else if (state == AutonState.shoot1 || state == AutonState.followPath1)
        {
            hoodbase = 0.473;
            vel = 1509;
        }
        else
        {
            hoodbase = 0.523 ;
            vel = 1570;
        }
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
        double dist = Math.hypot(goalx - p.getX(), goaly - p.getY());
        double shottime = timeA*dist*dist + timeB*dist + timeC;
        return new Pose(goalx - vx*shottime, goaly - vy*shottime, 0);
    }


    private void aim()
    {
        double tgtangle = 0;
        double clamped = clampturret();
        if (Math.abs(clamped - getturretdeg()) <= thresh) tgtangle = clamped;


        double cur = getturretdeg();
        double err = wrapangle(tgtangle - cur);


        if (Math.abs(err) <= tdead) { turret.setPower(0); return; }


        tpid.setPID(tkp, tki, tkd);
        double out = tpid.calculate(cur, tgtangle);
        double ff = Math.abs(err) > tffdead ? Math.copySign(tks, err) : 0.0;
        double power = out + ff;
        if (power > tmax) power = tmax;
        if (power < -tmax) power = -tmax;
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
        double batv = Math.max(10.5, vsensor.getVoltage());
        double ff = bangff * (targetvelocity / maxvel) * (12.0 / batv);
        double bb = fly2.getVelocity() < targetvelocity ? 1.0 : 0.0;
        double power = Math.min(1.0, bb + ff);
        fly1.setPower(power);
        fly2.setPower(power);
    }


    private double getturretdeg()
    {
        double angle = (tenc.getVoltage() / tenc.getMaxVoltage()) * 360.0 - tzero;
        return wrapasym(angle, thresh);
    }


    private double clampturret()
    {
        double rawangle;
        if (state == AutonState.shoot1 || state == AutonState.followPath1)
            rawangle = -149.29999999;
        else if (state == AutonState.shoot11 || state == AutonState.followPath11)
            rawangle = -35;
        else if (state == AutonState.shoot13 || state == AutonState.followPath13)
            rawangle = -88;
        else if (state == AutonState.followPath3 || state == AutonState.shoot3)
            rawangle = -67.47;
        else
            rawangle = -37.267;


        double d = wrapasym(rawangle, thresh);
        if (d > thresh2) return thresh2;
        if (d < -thresh) return -thresh;
        return d;
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
