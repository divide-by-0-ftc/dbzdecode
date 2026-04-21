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
public class BLUEAUTOV2 extends DbzOpMode {
    public static double servooff = 0.01;
    public static double push0 = 0.81, push3 = 0.22;
    public static double lockpos = 0.71;
    public static double holdopen = 0.8, holdclose = 0.467;
    public static double hooddefault = 0.5;

    public static double tzero = 194.5;
    public static double tkp = 0.03, tki = 0.0, tkd = 0.005;
    public static double tdead = 0.5, tmax = 1.0;
    public static double thresh = 140.0, thresh2 = 140.0;

    public static double vkF = 0.00022, vkVConst = 12.5;

    public static double gatex = 144 - 145.56, gatey = 61.28, gateh = 21;
    public static double startx = 144 - 114.2417, starty = 133.472;

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13;

        public Paths(Follower f) {
            Path1 = f.pathBuilder().addPath(new BezierLine(new Pose(144-111.417, 136.815), new Pose(144-98.149, 83.168))).setTangentHeadingInterpolation().build();
            Path2 = f.pathBuilder().addPath(new BezierCurve(new Pose(144-98.149, 83.168), new Pose(144-110.518, 62.028), new Pose(144-130.642, 58.482))).setTangentHeadingInterpolation().build();
            Path3 = f.pathBuilder().addPath(new BezierLine(new Pose(144-130.642, 58.482), new Pose(144-97.149, 77.168))).setTangentHeadingInterpolation().setReversed().build();
            Path4 = f.pathBuilder().addPath(new BezierCurve(new Pose(144-97.149, 77.168), new Pose(144-110.990, 60.361), new Pose(gatex, gatey))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180-gateh)).build();
            Path5 = f.pathBuilder().addPath(new BezierCurve(new Pose(gatex, gatey), new Pose(144-110.990, 60.361), new Pose(144-97.149, 77.168))).setLinearHeadingInterpolation(Math.toRadians(180-gateh), Math.toRadians(180)).build();
            Path6 = f.pathBuilder().addPath(new BezierCurve(new Pose(144-97.149, 77.168), new Pose(144-110.990, 60.361), new Pose(gatex, gatey))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180-gateh)).build();
            Path7 = f.pathBuilder().addPath(new BezierCurve(new Pose(gatex, gatey), new Pose(144-110.990, 60.361), new Pose(144-97.149, 77.168))).setLinearHeadingInterpolation(Math.toRadians(180-gateh), Math.toRadians(180)).build();
            Path8 = f.pathBuilder().addPath(new BezierCurve(new Pose(144-97.149, 77.168), new Pose(144-110.990, 60.361), new Pose(gatex, gatey))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180-gateh)).build();
            Path9 = f.pathBuilder().addPath(new BezierCurve(new Pose(gatex, gatey), new Pose(144-110.990, 60.361), new Pose(144-97.149, 84.566))).setLinearHeadingInterpolation(Math.toRadians(180-gateh), Math.toRadians(180)).build();
            Path10 = f.pathBuilder().addPath(new BezierLine(new Pose(144-97.149, 84.566), new Pose(144-128.573, 84.566))).setTangentHeadingInterpolation().build();
            Path11 = f.pathBuilder().addPath(new BezierLine(new Pose(144-128.573, 84.566), new Pose(144-95.149, 84.168))).setTangentHeadingInterpolation().setReversed().build();
            Path12 = f.pathBuilder().addPath(new BezierCurve(new Pose(144-95.149, 84.168), new Pose(144-110.251, 40.730), new Pose(144-138.546, 28.591))).setTangentHeadingInterpolation().build();
            Path13 = f.pathBuilder().addPath(new BezierLine(new Pose(144-138.546, 28.591), new Pose(144-96.5, 112))).setTangentHeadingInterpolation().setReversed().build();
        }
    }

    protected Servo rpush, lpush, hood, hold, blinkin;
    protected DcMotorEx intake, fly1, fly2, turret;
    private VoltageSensor vsensor;
    private AnalogInput tenc;
    private PIDController tpid;
    private Follower follower;
    private Paths paths;

    private enum AutonState { followPath1, shoot1, followPath2, followPath3, shoot3, followPath4, intakeWait1, followPath5, shoot5, followPath6, intakeWait2, followPath7, shoot7, followPath8, intakeWait3, followPath9, shoot9, followPath10, followPath11, shoot11, followPath12, followPath13, shoot13, done }
    private AutonState state = AutonState.followPath1;

    private ElapsedTime statetimer = new ElapsedTime();
    private double targetvelocity = 0, hoodbase = hooddefault;

    @Override
    public void opInit() {
        rpush = hardwareMap.get(Servo.class, "rightpushServo");
        lpush = hardwareMap.get(Servo.class, "leftpushServo");
        hood = hardwareMap.get(Servo.class, "hoodServo");
        hold = hardwareMap.get(Servo.class, "holdServo");
        blinkin = hardwareMap.get(Servo.class, "light");

        tenc = hardwareMap.get(AnalogInput.class, "turretEncoder");

        intake = robot.intakeMotor;
        fly1 = robot.outtake1Motor; fly2 = robot.outtake2Motor;
        fly1.setDirection(DcMotorEx.Direction.REVERSE);

        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        vsensor = hardwareMap.voltageSensor.iterator().next();
        tpid = new PIDController(tkp, tki, tkd);

        hood.setPosition(hooddefault);
        hold.setPosition(holdopen);
        lpush.setPosition(lockpos);
        rpush.setPosition(lockpos - servooff);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startx, starty, Math.toRadians(270)));
        paths = new Paths(follower);

        follower.followPath(paths.Path1, true);
        statetimer.reset();
    }

    @Override
    protected void opLoopHook() {

    }

    @Override
    public void opLoop() {
        follower.update();
        regressions();
        runflywheel();
        aim();

        switch (state) {
            case followPath1: if (!follower.isBusy()) { intake.setPower(1); startshoot(); statetimer.reset(); state = AutonState.shoot1; } break;
            case shoot1: if (statetimer.seconds() >= 0.5) { endshoot(); follower.followPath(paths.Path2, true); state = AutonState.followPath2; } break;
            case followPath2: hold.setPosition(holdclose); if (!follower.isBusy()) { follower.followPath(paths.Path3, true); intake.setPower(1); state = AutonState.followPath3; } break;
            case followPath3: hold.setPosition(holdopen); if (!follower.isBusy()) { intake.setPower(1); startshoot(); statetimer.reset(); state = AutonState.shoot3; } break;
            case shoot3: if (statetimer.seconds() >= 0.6) { endshoot(); intake.setPower(1); follower.followPath(paths.Path4, true); state = AutonState.followPath4; } break;
            case followPath4: intake.setPower(1); hold.setPosition(holdclose); if (!follower.isBusy()) { statetimer.reset(); state = AutonState.intakeWait1; } break;
            case intakeWait1: if (statetimer.seconds() >= 0.28) { follower.followPath(paths.Path5, true); state = AutonState.followPath5; } break;
            case followPath5: if (statetimer.seconds() > 0.5) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); intake.setPower(-1); } if (statetimer.seconds() > 0.8) hold.setPosition(holdopen); if (!follower.isBusy()) { startshoot(); statetimer.reset(); state = AutonState.shoot5; } break;
            case shoot5: if (statetimer.seconds() >= 0.5) { endshoot(); intake.setPower(1); follower.followPath(paths.Path6, true); state = AutonState.followPath6; } break;
            case followPath6: intake.setPower(1); hold.setPosition(holdclose); if (!follower.isBusy()) { statetimer.reset(); state = AutonState.intakeWait2; } break;
            case intakeWait2: if (statetimer.seconds() >= 1.0) { follower.followPath(paths.Path7, true); state = AutonState.followPath7; } break;
            case followPath7: if (statetimer.seconds() > 1.1) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); } if (statetimer.seconds() >= 1.3) { intake.setPower(-1); hold.setPosition(holdopen); } if (!follower.isBusy()) { startshoot(); intake.setPower(1); statetimer.reset(); state = AutonState.shoot7; } break;
            case shoot7: if (statetimer.seconds() >= 0.5) { endshoot(); intake.setPower(1); follower.followPath(paths.Path8, true); state = AutonState.followPath8; } break;
            case followPath8: intake.setPower(1); hold.setPosition(holdclose); if (!follower.isBusy()) { statetimer.reset(); state = AutonState.intakeWait3; } break;
            case intakeWait3: if (statetimer.seconds() >= 1.0) { follower.followPath(paths.Path9, true); state = AutonState.followPath9; } break;
            case followPath9: if (statetimer.seconds() > 1.1) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); } if (statetimer.seconds() >= 1.3) { intake.setPower(-1); hold.setPosition(holdopen); } if (!follower.isBusy()) { startshoot(); intake.setPower(1); statetimer.reset(); state = AutonState.shoot9; } break;
            case shoot9: if (statetimer.seconds() >= 0.5) { endshoot(); follower.followPath(paths.Path10, true); state = AutonState.followPath10; } break;
            case followPath10: intake.setPower(1); hold.setPosition(holdclose); if (!follower.isBusy()) { follower.followPath(paths.Path11, true); statetimer.reset(); state = AutonState.followPath11; } break;
            case followPath11: hold.setPosition(holdopen); if (!follower.isBusy()) { startshoot(); statetimer.reset(); state = AutonState.shoot11; } break;
            case shoot11: if (statetimer.seconds() >= 0.5) { endshoot(); follower.followPath(paths.Path12, true); hold.setPosition(holdclose); state = AutonState.followPath12; } break;
            case followPath12: if (!follower.isBusy()) { follower.followPath(paths.Path13, true); statetimer.reset(); state = AutonState.followPath13; } break;
            case followPath13: lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); if (statetimer.seconds() > 0.3) { intake.setPower(-1); hold.setPosition(holdopen); } if (!follower.isBusy()) { intake.setPower(1); startshoot(); statetimer.reset(); state = AutonState.shoot13; } break;
            case shoot13: if (statetimer.seconds() >= 0.5) { endshoot(); state = AutonState.done; } break;
            case done: intake.setPower(0); fly1.setPower(0); fly2.setPower(0); turret.setPower(0); break;
        }

        telemetry.addData("state", state);
        telemetry.addData("turret deg", getturretdeg());
        telemetry.update();
    }

    private void startshoot() {
        lpush.setPosition(push3);
        rpush.setPosition(push3 - servooff);
    }

    private void endshoot() {
        lpush.setPosition(push0);
        rpush.setPosition(push0 - servooff);
    }

    private void regressions() {
        if (state == AutonState.shoot13 || state == AutonState.followPath13) { hoodbase = 0.22; targetvelocity = 1350; }
        else if (state == AutonState.shoot1 || state == AutonState.followPath1) { hoodbase = 0.473; targetvelocity = 1509; }
        else { hoodbase = 0.523; targetvelocity = 1570; }
    }

    private void aim() {
        double tgt = clampturret();
        double cur = getturretdeg();
        double err = wrapangle(tgt - cur);

        if (Math.abs(err) <= tdead) {
            turret.setPower(0);
            return;
        }

        double out = tpid.calculate(cur, tgt);
        turret.setPower(Math.max(-tmax, Math.min(tmax, out)));
    }

    private void runflywheel() {
        if (targetvelocity <= 1.0) {
            fly1.setPower(0);
            fly2.setPower(0);
            return;
        }
        double vt = vkVConst / Math.max(10.5, vsensor.getVoltage());
        double power = fly2.getVelocity() < targetvelocity - 50 ? 1.0 : vkF * targetvelocity * vt;
        fly1.setPower(power);
        fly2.setPower(power);
    }

    private double getturretdeg() {
        double angle = (tenc.getVoltage() / tenc.getMaxVoltage()) * 360.0 - tzero;
        return wrapasym(angle, thresh);
    }

    private double clampturret() {
        double raw;
        if (state == AutonState.shoot1 || state == AutonState.followPath1) raw = -149.3;
        else if (state == AutonState.shoot11 || state == AutonState.followPath11) raw = -35;
        else if (state == AutonState.shoot13 || state == AutonState.followPath13) raw = -88;
        else if (state == AutonState.followPath3 || state == AutonState.shoot3) raw = -67.5;
        else raw = -37.3;

        double d = wrapasym(raw, thresh);
        if (d > thresh2) return thresh2;
        if (d < -thresh) return -thresh;
        return d;
    }

    private double wrapangle(double a) { return ((a + 180) % 360 + 360) % 360 - 180; }
    private double wrapasym(double a, double n) { return ((a + n) % 360 + 360) % 360 - n; }

    @Override public void opTeardown() { org.firstinspires.ftc.teamcode.tele.PoseCache.lastPose = follower.getPose(); }
}