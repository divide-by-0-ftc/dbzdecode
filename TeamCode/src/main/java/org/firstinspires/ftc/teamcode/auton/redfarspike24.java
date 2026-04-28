package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
@Autonomous(name = "redfarspike24")
public class redfarspike24 extends DbzOpMode {

    public static double servooff    = 0.01;
    public static double push0       = 0.81,  push3     = 0.22;
    public static double lockpos     = 0.71;
    public static double holdopen    = 0.8,   holdclose = 0.467;
    public static double hooddefault = 0.5;

    public static double tkp = 0.03,  tkd = 0.0015, tkv = 0.001;
    public static double tdead = 0.5, tmax = 1.0;
    public static double thresh = 140.0, thresh2 = 140.0, tzero = 191.0;
    public static double turretVelAlpha = 0.2;

    public static double vkF = 0.0002, vkBBThresh = 50.0, vkVConst = 12.0;

    public static double arch = 0.27;
    public static double arcv = 0.0;

    public static double intakeWait1Dur = 0.1;
    public static double intakeWait2Dur = 0.9;
    public static double intakeWait3Dur = 0.9;
    public static double intakeWait4Dur = 0.9;

    public static double pushLockDelay  = 0.4;
    public static double intakeRevDelay = 0.6;

    public static double goalx  = 0.0,            goaly  = 144.0;
    public static double startx = 114.2417, starty = 133.472;
    public static double gatex  = 145.56,   gatey  = 61.28,  gateh = 21;

    public static double dthresh  = 0.17, dthresh1 = 0.193, dthresh2 = 0.175;
    public static double sticky   = 0.2;
    public static double sensorConfirmSec = 0.3;

    public static double shootDur = 0.3;

    public static double dipamt   = 0.05;
    public static double dipdelay = 0.1;
    public static double dipdur   = 0.5;

    public static double turretBumpDeg = -5.0;
    public static double turretBumpDur = 1;

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5,
                Path6, Path7, Path8, Path9, Path10,
                Path11, Path12, Path13, Path14, Path15;

        public Paths(Follower f) {
            Path1  = f.pathBuilder().addPath(new BezierLine(new Pose(111.417, 136.815), new Pose(98.149, 83.168))).setTangentHeadingInterpolation().build();
            Path2  = f.pathBuilder().addPath(new BezierCurve(new Pose(98.149, 83.168), new Pose(110.518, 62.028), new Pose(133.642, 58.482))).setTangentHeadingInterpolation().build();
            Path3  = f.pathBuilder().addPath(new BezierLine(new Pose(130.642, 58.482), new Pose(100.149, 77.168))).setTangentHeadingInterpolation().setReversed().build();
            Path4  = f.pathBuilder().addPath(new BezierCurve(new Pose(97.149, 77.168), new Pose(115.990, 60.361), new Pose(gatex, gatey))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gateh)).build();
            Path5  = f.pathBuilder().addPath(new BezierCurve(new Pose(gatex, gatey), new Pose(110.990, 60.361), new Pose(102.149, 72.168))).setLinearHeadingInterpolation(Math.toRadians(gateh), Math.toRadians(0)).build();
            Path6  = f.pathBuilder().addPath(new BezierCurve(new Pose(102.149, 72.168), new Pose(110.990, 60.361), new Pose(gatex, gatey))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gateh)).build();
            Path7  = f.pathBuilder().addPath(new BezierCurve(new Pose(gatex, gatey), new Pose(110.990, 60.361), new Pose(102.149, 72.168))).setLinearHeadingInterpolation(Math.toRadians(gateh), Math.toRadians(0)).build();
            Path8  = f.pathBuilder().addPath(new BezierCurve(new Pose(102.149, 72.168), new Pose(110.990, 60.361), new Pose(gatex, gatey))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gateh)).build();
            Path9  = f.pathBuilder().addPath(new BezierCurve(new Pose(gatex, gatey), new Pose(110.990, 60.361), new Pose(102.149, 72.168))).setLinearHeadingInterpolation(Math.toRadians(gateh), Math.toRadians(0)).build();
            Path14 = f.pathBuilder().addPath(new BezierCurve(new Pose(102.149, 72.168), new Pose(110.990, 60.361), new Pose(gatex, gatey))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(gateh)).build();
            Path15 = f.pathBuilder().addPath(new BezierCurve(new Pose(gatex, gatey), new Pose(110.990, 60.361), new Pose(102.149, 79.566))).setLinearHeadingInterpolation(Math.toRadians(gateh), Math.toRadians(0)).build();
            Path10 = f.pathBuilder().addPath(new BezierLine(new Pose(102.149, 79.566), new Pose(128.573, 84.566))).setTangentHeadingInterpolation().build();
            Path11 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(128.573, 84.566),
                            new Pose(95.149, 84.168)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(275))
                    .build();
            Path12 = f.pathBuilder().addPath(new BezierCurve(new Pose(95.149, 84.168), new Pose(110.251, 38.730), new Pose(12, 28.591))).setTangentHeadingInterpolation().build();
            Path13 = f.pathBuilder().addPath(new BezierLine(new Pose(144-12, 28.591), new Pose(96.5, 112.0))).setTangentHeadingInterpolation().setReversed().build();
        }
    }

    private enum AutonState {
        followPath1, shoot1,
        followPath2,
        followPath3, shoot3,
        followPath4, intakeWait1,
        followPath5, shoot5,
        followPath6, intakeWait2,
        followPath7, shoot7,
        followPath8, intakeWait3,
        followPath9, shoot9,
        followPath14, intakeWait4,
        followPath15, shoot15,
        followPath10,
        followPath11, shoot11,
        followPath12,
        followPath13, shoot13,
        done
    }

    protected Servo       rpush, lpush, hood, hold;
    protected DcMotorEx   intake, fly1, fly2, turret;
    private   VoltageSensor vsensor;
    private   AnalogInput   tenc, d0, d1, d2;
    private   Follower      follower;
    private   Paths         paths;

    private AutonState  state       = AutonState.followPath1;
    private ElapsedTime statetimer  = new ElapsedTime();
    private ElapsedTime turretDt    = new ElapsedTime();
    private ElapsedTime st0         = new ElapsedTime();
    private ElapsedTime st1         = new ElapsedTime();
    private ElapsedTime st2         = new ElapsedTime();
    private ElapsedTime detecttimer = new ElapsedTime();
    private ElapsedTime revtimer    = new ElapsedTime();
    private ElapsedTime diptimer    = new ElapsedTime();
    private ElapsedTime bumpTimer   = new ElapsedTime();

    private final ElapsedTime autoTimer = new ElapsedTime();
    private boolean shot1Fired = false;

    private double  targetvelocity           = 0;
    private double  hoodbase                 = hooddefault;
    private double  lastTurretErr            = 0;
    private double  prevTargetDeg            = 0;
    private double  filteredDesiredTurretVel = 0;
    private double  cachedTurretDeg          = 0;
    private boolean latch0                   = false;
    private boolean latch1                   = false;
    private boolean latch2                   = false;
    private boolean prevdetect               = false;
    private boolean ballLocked               = false;
    private boolean reversing                = false;
    private boolean dipping                  = false;
    private boolean dipdone                  = false;
    private boolean turretBump               = false;

    @Override
    public void opInit() {
        rpush = hardwareMap.get(Servo.class, "rightpushServo");
        lpush = hardwareMap.get(Servo.class, "leftpushServo");
        hood  = hardwareMap.get(Servo.class, "hoodServo");
        hold  = hardwareMap.get(Servo.class, "holdServo");

        d0 = hardwareMap.get(AnalogInput.class, "distancez");
        d1 = hardwareMap.get(AnalogInput.class, "distance1");
        d2 = hardwareMap.get(AnalogInput.class, "distance2");

        intake = robot.intakeMotor;
        fly1   = robot.outtake1Motor;
        fly2   = robot.outtake2Motor;
        fly1.setDirection(DcMotorEx.Direction.REVERSE);

        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        tenc    = hardwareMap.get(AnalogInput.class, "turretEncoder");
        vsensor = hardwareMap.voltageSensor.iterator().next();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hood.setPosition(hooddefault);
        hold.setPosition(holdopen);
        lpush.setPosition(lockpos);
        rpush.setPosition(lockpos - servooff);

        follower = Constants2.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startx, starty, Math.toRadians(270)));
        paths = new Paths(follower);

        cachedTurretDeg = rawTurretDeg();
        prevTargetDeg   = cachedTurretDeg;
        lastTurretErr   = 0;
        turretDt.reset();

        follower.followPath(paths.Path1);
        statetimer.reset();
    }

    @Override protected void opLoopHook() {}

    @Override
    public void opLoop() {
        follower.update();
        cachedTurretDeg = rawTurretDeg();
        updateSensors();
        regressions();
        aim();
        runflywheel();

        switch (state) {
            case followPath1:
                if (!follower.isBusy()) { intake.setPower(1); startshoot(); statetimer.reset(); state = AutonState.shoot1; }
                break;
            case shoot1:
                if (statetimer.seconds() >= shootDur) { endshoot(); follower.followPath(paths.Path2); state = AutonState.followPath2; }
                break;
            case followPath2:
                hold.setPosition(holdclose);
                if (!follower.isBusy()) { follower.followPath(paths.Path3); intake.setPower(1); state = AutonState.followPath3; }
                break;
            case followPath3:
                hold.setPosition(holdopen);
                if (!follower.isBusy()) { intake.setPower(1); startshoot(); statetimer.reset(); state = AutonState.shoot3; }
                break;
            case shoot3:
                if (statetimer.seconds() >= shootDur) { endshoot(); intake.setPower(1); follower.followPath(paths.Path4); state = AutonState.followPath4; }
                break;
            case followPath4:
                intake.setPower(1); hold.setPosition(holdclose);
                if (!follower.isBusy()) { resetBall(); statetimer.reset(); state = AutonState.intakeWait1; }
                break;
            case intakeWait1:
                runBallDetect();
                if (statetimer.seconds() >= intakeWait1Dur) { follower.followPath(paths.Path5); statetimer.reset(); state = AutonState.followPath5; }
                break;
            case followPath5:
                if (statetimer.seconds() > pushLockDelay) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); intake.setPower(-1); }
                if (statetimer.seconds() > intakeRevDelay) { hold.setPosition(holdopen); }
                if (!follower.isBusy()) { startshoot(); statetimer.reset(); state = AutonState.shoot5; }
                break;
            case shoot5:
                if (statetimer.seconds() >= 0.1 && !turretBump) { turretBump = true; bumpTimer.reset(); }
                if (statetimer.seconds() >= shootDur) { endshoot(); intake.setPower(1); follower.followPath(paths.Path6); state = AutonState.followPath6; }
                break;
            case followPath6:
                intake.setPower(1); hold.setPosition(holdclose);
                if (!follower.isBusy() || statetimer.seconds() > 3.0) { statetimer.reset(); state = AutonState.intakeWait2; }
                break;
            case intakeWait2:
                intake.setPower(1);
                if (statetimer.seconds() >= intakeWait2Dur) { follower.followPath(paths.Path7); statetimer.reset(); state = AutonState.followPath7; }
                break;
            case followPath7:
                if (statetimer.seconds() > pushLockDelay) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= intakeRevDelay) { intake.setPower(-1); hold.setPosition(holdopen); }
                if (!follower.isBusy()) { startshoot(); intake.setPower(1); statetimer.reset(); state = AutonState.shoot7; }
                break;
            case shoot7:
                if (statetimer.seconds() >= 0.1 && !turretBump) { turretBump = true; bumpTimer.reset(); }
                if (statetimer.seconds() >= shootDur) { endshoot(); intake.setPower(1); follower.followPath(paths.Path8); state = AutonState.followPath8; }
                break;
            case followPath8:
                intake.setPower(1); hold.setPosition(holdclose);
                if (!follower.isBusy() || statetimer.seconds() > 3.0) { statetimer.reset(); state = AutonState.intakeWait3; }
                break;
            case intakeWait3:
                intake.setPower(1);
                if (statetimer.seconds() >= intakeWait3Dur) { follower.followPath(paths.Path9); statetimer.reset(); state = AutonState.followPath9; }
                break;
            case followPath9:
                if (statetimer.seconds() > pushLockDelay) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= intakeRevDelay) { intake.setPower(-1); hold.setPosition(holdopen); }
                if (!follower.isBusy()) { startshoot(); intake.setPower(1); statetimer.reset(); state = AutonState.shoot9; }
                break;
            case shoot9:
                if (statetimer.seconds() >= 0.1 && !turretBump) { turretBump = true; bumpTimer.reset(); }
                if (statetimer.seconds() >= shootDur) { endshoot(); intake.setPower(1); follower.followPath(paths.Path14); state = AutonState.followPath14; }
                break;
            case followPath14:
                intake.setPower(1); hold.setPosition(holdclose);
                if (!follower.isBusy() || statetimer.seconds() > 3.0) { statetimer.reset(); state = AutonState.intakeWait4; }
                break;
            case intakeWait4:
                intake.setPower(1);
                if (statetimer.seconds() >= intakeWait4Dur) { follower.followPath(paths.Path15); statetimer.reset(); state = AutonState.followPath15; }
                break;
            case followPath15:
                if (statetimer.seconds() > pushLockDelay) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() >= intakeRevDelay) { intake.setPower(-1); hold.setPosition(holdopen); }
                if (!follower.isBusy()) { startshoot(); intake.setPower(1); statetimer.reset(); state = AutonState.shoot15; }
                break;
            case shoot15:
                if (statetimer.seconds() >= 0.1 && !turretBump) { turretBump = true; bumpTimer.reset(); }
                if (statetimer.seconds() >= shootDur) { endshoot(); follower.followPath(paths.Path10); state = AutonState.followPath10; }
                break;
            case followPath10:
                intake.setPower(1); hold.setPosition(holdclose);
                if (!follower.isBusy()) { follower.followPath(paths.Path11); statetimer.reset(); state = AutonState.followPath11; }
                break;
            case followPath11:
                hold.setPosition(holdopen);
                if (!follower.isBusy()) { startshoot(); statetimer.reset(); state = AutonState.shoot11; }
                break;
            case shoot11:
                if (statetimer.seconds() >= 0.5) { endshoot(); follower.followPath(paths.Path12); hold.setPosition(holdclose); state = AutonState.followPath12; }
                break;
            case followPath12:
                if (!follower.isBusy()) { follower.followPath(paths.Path13, true); statetimer.reset(); state = AutonState.followPath13; }
                break;
            case followPath13:
                if (statetimer.seconds() > 0.5) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (statetimer.seconds() > 0.7) { intake.setPower(-1); hold.setPosition(holdopen); }
                if (statetimer.seconds() > 2.0) { intake.setPower(1); startshoot(); statetimer.reset(); state = AutonState.shoot13; }
                break;
            case shoot13:
                if (statetimer.seconds() >= 0.3) { endshoot(); state = AutonState.done; }
                break;
            case done:
                break;
        }

        dipshot();

        telemetry.addData("state",          state);
        telemetry.addData("turret target",  getdesiredturretdeg());
        telemetry.addData("turret current", cachedTurretDeg);
        telemetry.addData("fly target",     targetvelocity);
        telemetry.addData("fly current",    fly2.getVelocity());
        telemetry.addData("d0",             d0.getVoltage());
        telemetry.addData("d1",             d1.getVoltage());
        telemetry.addData("d2",             d2.getVoltage());
        telemetry.addData("ballLocked",     ballLocked);
        telemetry.addData("turretBump",     turretBump);
        Pose p = follower.getPose();
        if (p != null) { telemetry.addData("dist", Math.hypot(goalx - p.getX(), goaly - p.getY())); telemetry.addData("hood", hoodbase); }
        telemetry.update();
    }

    private void resetBall() { prevdetect = false; ballLocked = false; reversing = false; }

    private void updateSensors() {
        if (d0.getVoltage() < dthresh)  { latch0 = true; st0.reset(); }
        if (d1.getVoltage() < dthresh1) { latch1 = true; st1.reset(); }
        if (d2.getVoltage() < dthresh2) { latch2 = true; st2.reset(); }
        if (st0.seconds() > sticky) latch0 = false;
        if (st1.seconds() > sticky) latch1 = false;
        if (st2.seconds() > sticky) latch2 = false;
    }

    private void runBallDetect() {
        boolean hit = latch0 && latch1 && latch2;
        if (!reversing && !ballLocked) {
            if (hit) {
                if (!prevdetect) { detecttimer.reset(); prevdetect = true; }
                if (detecttimer.seconds() >= sensorConfirmSec) {
                    latch0 = latch1 = latch2 = false; prevdetect = false;
                    lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff);
                    intake.setPower(-1); revtimer.reset(); reversing = true;
                }
            } else { prevdetect = false; intake.setPower(1); }
        }
        if (reversing) {
            lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff);
            if      (revtimer.seconds() < 0.5)  intake.setPower(-1);
            else if (revtimer.seconds() < 0.85) intake.setPower(1);
            else { intake.setPower(1); reversing = false; ballLocked = true; }
        }
        if (ballLocked) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); intake.setPower(1); }
    }

    private void dipshot() {
        boolean shooting = state == AutonState.shoot1  || state == AutonState.shoot3  ||
                state == AutonState.shoot5  || state == AutonState.shoot7  ||
                state == AutonState.shoot9  || state == AutonState.shoot11 ||
                state == AutonState.shoot13 || state == AutonState.shoot15;
        if (shooting && !dipping && !dipdone) { dipping = true; diptimer.reset(); }
        if (!shooting) { dipping = false; dipdone = false; hood.setPosition(hoodbase); return; }
        if (dipping) {
            double t = diptimer.seconds();
            if      (t < dipdelay)           hood.setPosition(hoodbase);
            else if (t < dipdelay + dipdur)  hood.setPosition(Math.max(0.0, hoodbase - dipamt));
            else { hood.setPosition(hoodbase); dipping = false; dipdone = true; }
        } else { hood.setPosition(hoodbase); }
    }

    private void regressions() {
        double rawHood, rawVel;
        switch (state) {
            case shoot1:  case followPath1:  rawHood = 0.69; rawVel = 2410; break;
            case shoot3:  case followPath3:  rawHood = 0.70; rawVel = 2446; break;
            case shoot13: case followPath13: rawHood = 0.50; rawVel = 2050; break;
            case shoot11: case followPath11: rawHood = 0.69; rawVel = 2415; break;
            case shoot15: case followPath15: rawHood = 0.69; rawVel = 2413; break;
            default:                         rawHood = 0.70; rawVel = 2461; break;
        }
        hoodbase       = rawHood - arch;
        targetvelocity = rawVel  - arcv;
    }

    private void aim() {
        double tgt = clampturret();
        double cur = cachedTurretDeg;
        double err = wrapangle(tgt - cur);

        double dt = turretDt.seconds();
        turretDt.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        double rawDesiredVel = wrapangle(tgt - prevTargetDeg) / dt;
        filteredDesiredTurretVel = filteredDesiredTurretVel*(1-turretVelAlpha) + rawDesiredVel*turretVelAlpha;

        if (Math.abs(err) <= tdead) { prevTargetDeg = tgt; lastTurretErr = err; turret.setPower(0); return; }

        double errDiff  = wrapangle(err - lastTurretErr);
        double feedback = tkp * err + tkd * (errDiff / dt);
        double velFF    = tkv * filteredDesiredTurretVel;

        lastTurretErr = err; prevTargetDeg = tgt;
        turret.setPower(Math.max(-tmax, Math.min(tmax, feedback + velFF)));
    }

    private void runflywheel() {
        if (targetvelocity <= 1.0) { fly1.setPower(0); fly2.setPower(0); return; }
        double vt    = vkVConst / Math.max(10.5, vsensor.getVoltage());
        double power = fly2.getVelocity() < targetvelocity - vkBBThresh ? vt : vkF * targetvelocity * vt;
        fly1.setPower(Math.min(1, Math.max(0, power)));
        fly2.setPower(Math.min(1, Math.max(0, power)));
    }

    private double rawTurretDeg() {
        return wrapasym((tenc.getVoltage() / tenc.getMaxVoltage()) * 360.0 - tzero, thresh);
    }

    private double getdesiredturretdeg() {
        double base;
        switch (state) {
            case shoot1:  case followPath1: case followPath2:  base =  -15.8; break;
            case shoot3:  case followPath3: case followPath4:  base = -104.5; break;
            case shoot13: case followPath13: base =  -94; break;
            case shoot11: case followPath11: base = -40; break;
            case shoot15: case followPath15: base = -132.5; break;
            default:                         base = -132.5; break;
        }
        if (turretBump) {
            if (bumpTimer.seconds() < turretBumpDur) return base + turretBumpDeg;
            else turretBump = false;
        }
        return base;
    }

    private double clampturret() {
        double d = wrapangle(getdesiredturretdeg());
        if (d >  thresh2) return  thresh2;
        if (d < -thresh)  return -thresh;
        return d;
    }

    private void startshoot() { lpush.setPosition(push3); rpush.setPosition(push3 - servooff); }
    private void endshoot()   { lpush.setPosition(push0); rpush.setPosition(push0 - servooff); }

    private double wrapangle(double a)          { return ((a + 180) % 360 + 360) % 360 - 180; }
    private double wrapasym(double a, double n) { return ((a + n)   % 360 + 360) % 360 - n;   }

    @Override
    public void opTeardown() {
        org.firstinspires.ftc.teamcode.tele.PoseCache.lastPose = follower.getPose();
    }
}