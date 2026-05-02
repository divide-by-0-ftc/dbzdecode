package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.auton.Tuning.draw;
import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.auton.ConstantsTele;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

import java.util.List;

@Config
@TeleOp(name = "REDV5")
public class REDV5 extends DbzOpMode {

    private enum TurretState { NORMAL, CENTERING }
    private enum BallState   { IDLE, REVERSING, LOCKED }

    public static double push0 = 0.85, push1 = 0.67, push2 = 0.47, push3 = 0.22;
    public static double lockpos = 0.73, twitch = 0.8, servooff = 0.035;
    public static double shot1 = 300, shot2 = 600, shotret = 1000;
    public static double holdopen = 0.8, holdclose = 0.467;
    public static double hooddefault = 0.3;
    public static double dipamt = 0.00, dipdelay = 0.1, dipdur = 1.5;

    public static double dthresh = 0.16, dthresh1 = 0.163, dthresh2 = 0.175;
    private boolean lastrup        = false, lastldown       = false;
    public static double sticky = 0.15;

    public static double[] lutD = {51.5,  60.9,  71.95,  80.3,  87.1,  97.1, 109.5};
    public static double[] lutH = {0.10,  0.28,  0.40,   0.45,  0.50,  0.52,  0.55};
    public static double[] lutV = {1270, 1400,  1470,   1520,  1560,  1630,  1730};


    public static double manualvel = 0;
    public static double veloff = 10;
    public static double timea = 0.00002, timeb = 0.004, timec = 0.25;

    public static double goalx = 144, goaly = 144;

    public static double tkp = 0.03, tkd = 0.0015, tkv = 0.00, tks = 0.0, tffdead = 0.0;
    public static double tdead = 0.0, tmax = 1.0, toff = 2.0;
    public static double thresh = 170, thresh2 = 170, tzero = 180;
    public static double turretVelAlpha = 0.2;

    public static double vkF = 0.00038, vkBBThresh = 50.0, vkVConst = 12;

    public static double sotmDelay = 0.1, sotmMinVel = 1.5, sotmScale = 0.2;
    public static double sotmVelAlpha = 0.3;

    public static double brakeWait = 0.3, brakeShootDelay = 0.0;

    public static double kSigmaD  = 0.002309;
    public static double arch     = 0.0;
    public static double arcv     = 0;
    public static double kSigmaLL = 25.9938;
    public static boolean llEnabled = true;
    public static double llTaMin = 0.05, llMaxJump = 40.0, llCooldown = 1.5;
    public static double llMaxDriveV = 2.0, llMaxRotV = 1.0, llMaxTurretV = 5.0;
    public static double xoffset = 0.0, yoffset = 6.0;
    public static double sotg = 1;

    protected Servo       rpush, lpush, hood, hold, blinkin;
    protected DcMotorEx   intake, fly1, fly2, turret;
    protected DcMotorEx   fl, fr, bl, br;
    private   VoltageSensor vsensor;
    private   AnalogInput   tenc, d0, d1, d2;

    private   List<LynxModule> hubs;

    private final ElapsedTime intaketimer = new ElapsedTime();
    private final ElapsedTime detecttimer = new ElapsedTime();
    private final ElapsedTime diptimer    = new ElapsedTime();
    private final ElapsedTime holdtimer   = new ElapsedTime();
    private final ElapsedTime revtimer    = new ElapsedTime();
    private final ElapsedTime braketimer  = new ElapsedTime();
    private final ElapsedTime st0         = new ElapsedTime();
    private final ElapsedTime st1         = new ElapsedTime();
    private final ElapsedTime st2         = new ElapsedTime();
    private final ElapsedTime llCoolTimer = new ElapsedTime();
    private final ElapsedTime turretDt    = new ElapsedTime();
    private final ElapsedTime kDtTimer    = new ElapsedTime();

    private TurretState turretstate = TurretState.NORMAL;
    private BallState   ballstate   = BallState.IDLE;

    private boolean shoot         = false, lastshoot    = false, hasShot      = false;
    private boolean fastmode      = true;
    private boolean autohood      = true,  lasta        = false;
    private boolean aiming        = true,  lastaim      = false;
    private boolean sotmActive    = false, lastsotm     = false;
    private boolean intakefwd     = true, intakerev    = false;
    private boolean lastlb        = false, lastrb       = false;

    private boolean lastlb2        = false, lastrb2      = false;
    private boolean lastr1        = false, lastl1       = false;
    private boolean lastr2        = false, lastl2      = false;
    private boolean lastdpadup2   = false, lastdpaddn2  = false;
    private boolean lastB         = false;
    private boolean lastB2        = false;
    private boolean lastTrig2     = false;
    private boolean latch0        = false, latch1       = false, latch2       = false;
    private boolean prevdetect    = false, ballslocked  = false;
    private boolean holdoverride  = false, holdwait     = false;
    private boolean dipping       = false, dipdone      = false;
    private boolean turretontarget= false, velontarget  = false;
    private boolean braking       = false;
    private boolean lastBrakeState= false;
    private boolean slowShoot2    = false;

    private int         slowStep        = 0;
    private final ElapsedTime slowShotTimer  = new ElapsedTime();
    private boolean slowReturning = false;

    private double holdpos              = holdclose;
    private double hoodbase             = hooddefault;
    private double targetvelocity       = 0;
    private double turretoffset         = 0;
    private double lastlightpos         = -1;
    private double targetdeg            = 0, currentdeg = 0;
    private double flytarget            = 0, flycurrent = 0;
    private double lastTurretErr        = 0;
    private double cachedTurretDeg      = 0;
    private double prevTargetDeg        = 0;
    private double filteredDesiredTurretVel = 0;
    private Pose   cachedVgoal          = null;

    private double svx = 0, svy = 0;
    private long   lastSotmMs = 0;

    private double  kdX = 0, kdY = 0;
    private double  kpX = 1, kpY = 1;

    private double  llLastTurretDeg  = 0, llLastHeadingRad = 0;
    private long    llLastTimeMs     = 0;
    private double  llTurretVelDeg   = 0, llRotVelRad      = 0;
    private double  llX = 0, llY = 0, llTa = 0, llJump = 0;
    private boolean llValid   = false, llApplied = false;
    private String  llStatus  = "IDLE";

    private int loopCount = 0;

    public static Follower follower;
    @IgnoreConfigurable
    public static TelemetryManager telemetrym;


    @Override
    public void opInit() {



        rpush   = hardwareMap.get(Servo.class, "rightpushServo");
        lpush   = hardwareMap.get(Servo.class, "leftpushServo");
        hood    = hardwareMap.get(Servo.class, "hoodServo");
        hold    = hardwareMap.get(Servo.class, "holdServo");
        blinkin = hardwareMap.get(Servo.class, "light");

        d0 = hardwareMap.get(AnalogInput.class, "distancez");
        d1 = hardwareMap.get(AnalogInput.class, "distance1");
        d2 = hardwareMap.get(AnalogInput.class, "distance2");

        hood.setPosition(hoodbase);
        hold.setPosition(holdpos);
        lpush.setPosition(push0);
        rpush.setPosition(push0 - servooff);

        intake = robot.intakeMotor;
        fly1   = robot.outtake1Motor;
        fly2   = robot.outtake2Motor;

        fly1.setDirection(DcMotorEx.Direction.REVERSE);
        fly2.setDirection(DcMotorEx.Direction.FORWARD);
        fly1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        tenc = hardwareMap.get(AnalogInput.class, "turretEncoder");

        vsensor = hardwareMap.voltageSensor.iterator().next();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        telemetry = new com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetrym = PanelsTelemetry.INSTANCE.getTelemetry();



        follower = ConstantsTele.createFollower(hardwareMap);
        follower.setStartingPose(PoseCache.lastPose);



        llCoolTimer.reset();
        kDtTimer.reset();
        llLastTimeMs = System.currentTimeMillis();
        lastSotmMs   = System.currentTimeMillis();
        turretDt.reset();

        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.update();
        follower.update();
        if (follower.getCurrentPath() != null) drawOnlyCurrent();
        follower.startTeleopDrive();
    }


    @Override
    public void opLoop() {

        for (LynxModule h : hubs) h.clearBulkCache();



        cachedTurretDeg = wrapasym((tenc.getVoltage() / tenc.getMaxVoltage()) * 360.0 - tzero, thresh);
        Pose robotPose  = follower.getPose();
        cachedVgoal     = (robotPose != null) ? virtualgoal(robotPose) : new Pose(goalx, goaly, 0);

        boolean abtn = gamepad1.a;
        if (abtn && !lasta) autohood = !autohood;
        lasta = abtn;


        if (gamepad1.dpad_right && !lastr1) turretoffset -= toff;
        if (gamepad1.dpad_left  && !lastl1) turretoffset += toff;
        lastr1 = gamepad1.dpad_right;
        lastl1 = gamepad1.dpad_left;


        if (gamepad2.dpad_right && !lastr2) turretoffset -= toff;
        if (gamepad2.dpad_left  && !lastl2) turretoffset += toff;
        lastr2 = gamepad2.dpad_right;
        lastl2 = gamepad2.dpad_left;

        if (gamepad2.dpad_down && !lastldown) arcv -= veloff;
        if (gamepad2.dpad_up  && !lastrup) arcv += veloff;
        lastrup = gamepad2.dpad_down;
        lastldown = gamepad2.dpad_up;


        boolean b = gamepad2.b;
        if (b && !lastB) { slowShoot2 = !slowShoot2; slowStep = 0; slowReturning = false; if (!slowShoot2) { braking = false; follower.startTeleopDrive(); } }
        lastB = b;


        if (slowShoot2) {
            boolean s1 = gamepad2.left_bumper;

            if (!slowReturning && s1 && !lastlb2) {
                slowStep++;
                if (slowStep == 1) {
                    lpush.setPosition(push1); rpush.setPosition(push1 - servooff);
                } else if (slowStep == 2) {
                    lpush.setPosition(push2); rpush.setPosition(push2 - servooff);
                } else {
                    lpush.setPosition(push3); rpush.setPosition(push3 - servooff);
                    slowShotTimer.reset(); slowReturning = true;
                    ballslocked = false; ballstate = BallState.IDLE;
                }
            }
            if (slowReturning && slowShotTimer.milliseconds() > 500) {
                lpush.setPosition(push0); rpush.setPosition(push0 - servooff);
                hold.setPosition(holdclose);
                slowReturning = false; slowStep = 0;
                intake.setPower(1); intakefwd = true;
                holdoverride = false;
                holdpos = holdclose;
            }
            lastlb2 = s1;
        }

        boolean sotmbtn = gamepad2.x;
        if (sotmbtn && !lastsotm) sotmActive = !sotmActive;
        lastsotm = sotmbtn;

        double mult = gamepad1.left_trigger > 0.1 ? 0.3 : 1.0;
        boolean trig = dbzGamepad1.right_trigger > 0.1;

        if (trig && !lastshoot && !shoot && !braking && !slowShoot2) {
            braking = true;
            braketimer.reset();
        }
        if (slowShoot2) { if (braking) { follower.startTeleopDrive(); } braking = false; hasShot = false; lastBrakeState = false; }

        boolean wantBrake = braking || shoot;
        if (wantBrake != lastBrakeState) {
            DcMotorEx.ZeroPowerBehavior zpb = wantBrake
                    ? DcMotorEx.ZeroPowerBehavior.BRAKE
                    : DcMotorEx.ZeroPowerBehavior.FLOAT;
            fl.setZeroPowerBehavior(zpb); fr.setZeroPowerBehavior(zpb);
            bl.setZeroPowerBehavior(zpb); br.setZeroPowerBehavior(zpb);
            lastBrakeState = wantBrake;
        }

        if (braking) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y  * sotg,
                    -gamepad1.left_stick_x  * sotg,
                    -gamepad1.right_stick_x * sotg,
                    true);
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y  * mult,
                    -gamepad1.left_stick_x  * mult,
                    -gamepad1.right_stick_x * mult,
                    true);
        }

        follower.update();

        if (dbzGamepad1.x) { follower.setPose(new Pose(131, 80, Math.toRadians(0)));  turretoffset = 0; }
        if (dbzGamepad1.y) { follower.setPose(new Pose(9.76378, 8.661, Math.toRadians(0))); turretoffset = 0; }

        regressions();
        runballdetection();
        shootfast();
        activeintake();
        updatehold();
        dipshot();
        aim();
        runflywheel();
        updatelights();

        if (robotPose != null) {
            telemetrym.addData("dist", Math.hypot(goalx - robotPose.getX(), goaly - robotPose.getY()));
        }

        if (follower.getCurrentPath() != null) draw();

        if (loopCount++ % 3 == 0) {
            sendtelem();
            telemetrym.update(telemetry);
            telemetry.update();
        }
    }





    private void runballdetection() {

        if (d0.getVoltage() < dthresh)  { latch0 = true; st0.reset(); }
        if (d1.getVoltage() < dthresh1) { latch1 = true; st1.reset(); }
        if (d2.getVoltage() < dthresh2) { latch2 = true; st2.reset(); }

        if (st0.seconds() > sticky) latch0 = false;
        if (st1.seconds() > sticky) latch1 = false;
        if (st2.seconds() > sticky) latch2 = false;

        boolean hit = latch0 && latch1 && latch2;

        switch (ballstate) {
            case IDLE:
                if (hit && !shoot) {
                    if (!prevdetect) { detecttimer.reset(); prevdetect = true; }
                    if (detecttimer.seconds() >= 0.2) {
                        latch0 = latch1 = latch2 = false;
                        holdoverride = false; holdpos = holdclose;
                        lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff);
                        intake.setPower(-1); revtimer.reset();
                        ballstate = BallState.REVERSING; prevdetect = false; ballslocked = true;
                    }
                } else if (!hit) {
                    prevdetect = false; ballslocked = false;
                }
                break;

            case REVERSING:
                holdoverride = false; holdpos = holdclose;
                if (!shoot && revtimer.seconds() < 0.5) {
                    lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); intake.setPower(-1);
                }
                if (revtimer.seconds() >= 0.5 && !shoot && !slowShoot2) {
                    intake.setPower(1); lpush.setPosition(twitch); rpush.setPosition(twitch - servooff);
                }
                if (revtimer.seconds() >= 0.85 && !shoot && !slowShoot2) {
                    intake.setPower(1); lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff);
                    ballstate = BallState.LOCKED;
                }
                break;

            case LOCKED:
                if (!shoot) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (shoot)  intake.setPower(1);
                break;
        }
    }


    private double lerp(double[] xs, double[] ys, double x) {
        if (x <= xs[0])             return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
        for (int i = 0; i < xs.length - 1; i++) {
            if (x <= xs[i + 1]) {
                double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return ys[ys.length - 1];
    }

    private void regressions() {
        if (cachedVgoal == null || follower.getPose() == null) return;
        if (!autohood) { hoodbase = hooddefault; targetvelocity = manualvel; return; }

        Pose   p    = follower.getPose();
        double dist = Math.hypot(cachedVgoal.getX() - p.getX(), cachedVgoal.getY() - p.getY());

        hoodbase       = Math.max(0.0, lerp(lutD, lutH, dist)) - arch;
        targetvelocity = Math.max(1250, lerp(lutD, lutV, dist)) - arcv;

        targetvelocity = Math.min(targetvelocity, 1750);
    }


    private void dipshot() {
        if (shoot && !dipping && !dipdone) { dipping = true; diptimer.reset(); }
        if (!shoot) { dipping = false; dipdone = false; hood.setPosition(hoodbase); return; }
        if (dipping) {
            double t = diptimer.seconds();
            if      (t < dipdelay)           hood.setPosition(hoodbase);
            else if (t < dipdelay + dipdur)  hood.setPosition(Math.max(0.0, hoodbase - dipamt));
            else { hood.setPosition(hoodbase); dipping = false; dipdone = true; }
        }
    }


    private Pose virtualgoal(Pose p) {

        com.pedropathing.math.Vector vel = follower.getVelocity();
        double rawVx = vel != null ? vel.getXComponent() : 0;
        double rawVy = vel != null ? vel.getYComponent() : 0;

        svx = svx*(1-sotmVelAlpha) + rawVx*sotmVelAlpha;
        svy = svy*(1-sotmVelAlpha) + rawVy*sotmVelAlpha;

        if (!sotmActive || Math.hypot(svx, svy) < sotmMinVel) return new Pose(goalx, goaly, 0);

        double dist1  = Math.hypot(goalx - p.getX(), goaly - p.getY());
        double tTotal = timea*dist1*dist1 + timeb*dist1 + timec + sotmDelay;
        double vgX    = goalx - svx*tTotal*sotmScale;
        double vgY    = goaly - svy*tTotal*sotmScale;

        double dist2 = Math.hypot(vgX - p.getX(), vgY - p.getY());
        tTotal = timea*dist2*dist2 + timeb*dist2 + timec + sotmDelay;
        vgX    = goalx - svx*tTotal*sotmScale;
        vgY    = goaly - svy*tTotal*sotmScale;

        return new Pose(vgX, vgY, 0);
    }


    private void fireShot() {
        if (slowShoot2) return;

        boolean had3      = ballslocked;
        boolean holdready = hold.getPosition() >= holdopen - 0.01;
        ballslocked = false; holdoverride = false;
        intaketimer.reset(); shoot = true; dipping = false; dipdone = false;

        if (!fastmode) {
            holdpos = holdclose;
            if (had3 && !holdready) { holdwait = true; holdtimer.reset(); }
            else { holdwait = false; lpush.setPosition(push1); rpush.setPosition(push1 - servooff); intaketimer.reset(); }
        } else {
            holdpos = holdopen;
            if (had3 && !holdready) { holdwait = true; holdtimer.reset(); }
            else { holdwait = false; lpush.setPosition(push3); rpush.setPosition(push3 - servooff); intaketimer.reset(); }
        }
    }

    private void shootfast() {
        if (slowShoot2) return;

        boolean trig = dbzGamepad1.right_trigger > 0.1;

        if (braking) {
            if (!hasShot && braketimer.seconds() >= brakeShootDelay) { fireShot(); hasShot = true; }
            if (braketimer.seconds() >= brakeWait) { braking = false; hasShot = false; follower.startTeleopDrive(); }
        }

        if (!fastmode) {
            if (shoot && holdwait && holdtimer.milliseconds() >= 200) {
                holdwait = false; lpush.setPosition(push1); rpush.setPosition(push1 - servooff); intaketimer.reset();
            } else if (shoot && !holdwait) {
                double ms = intaketimer.milliseconds();
                if      (ms > shotret) { lpush.setPosition(push0); rpush.setPosition(push0 - servooff); holdpos = holdclose; shoot = false; resetshot(); }
                else if (ms > shot2)   { lpush.setPosition(push3); rpush.setPosition(push3 - servooff); }
                else if (ms > shot1)   { lpush.setPosition(push2); rpush.setPosition(push2 - servooff); }
            }
        } else {
            if (shoot && holdwait && holdtimer.milliseconds() >= 200) {
                holdwait = false; lpush.setPosition(push3); rpush.setPosition(push3 - servooff); intaketimer.reset();
            } else if (shoot && !holdwait && intaketimer.milliseconds() > 500) {
                lpush.setPosition(push0); rpush.setPosition(push0 - servooff); holdpos = holdclose; shoot = false; resetshot();
            }
            if (!shoot && !braking) intaketimer.reset();
        }

        lastshoot = trig;
    }

    private void resetshot() {
        if (slowShoot2) return;
        ballslocked = false; ballstate = BallState.IDLE;
        holdoverride = false; holdpos = holdclose; holdwait = false;
        lpush.setPosition(push0); rpush.setPosition(push0 - servooff);
        intakefwd = true; intakerev = false; intake.setPower(1);
        follower.startTeleopDrive();
    }


    private void activeintake() {


        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        if (shoot)                            { intake.setPower(0); lastrb = rb; lastlb = lb; return; }
        if (ballstate == BallState.REVERSING) { lastrb = rb; lastlb = lb; return; }

        if (rb && !lastrb) { intakefwd = !intakefwd; intakerev = false; }
        if (lb && !lastlb) { intakerev = !intakerev; intakefwd = false; }

        if (intakerev) {
            intake.setPower(-1);
            if (!slowShoot2) { lpush.setPosition(push0); rpush.setPosition(push0 - servooff); }
            ballstate = BallState.IDLE; ballslocked = false;
            lastrb = rb; lastlb = lb; return;
        }

        if (ballslocked) { intake.setPower(-1); lastrb = rb; lastlb = lb; return; }

        intake.setPower(intakefwd ? 1 : 0);
        if (!ballslocked && !shoot && !slowShoot2) { lpush.setPosition(push0); rpush.setPosition(push0 - servooff); }
        lastrb = rb; lastlb = lb;
    }

    private void updatehold() {
        if (!holdoverride) {
            if      (shoot)                            holdpos = holdopen;
            else if (ballstate == BallState.REVERSING) holdpos = revtimer.seconds() >= 0.5 ? holdopen : holdclose;
            else if (ballstate == BallState.LOCKED)    holdpos = holdopen;
        }
        hold.setPosition(holdpos);
    }


    private void aim() {

        boolean aimBtn = gamepad1.dpad_up;
        if (aimBtn && !lastaim) {
            aiming = !aiming;
            lastTurretErr = 0; prevTargetDeg = getturretdeg(); filteredDesiredTurretVel = 0; turretDt.reset();
        }
        lastaim = aimBtn;

        double tgt = !aiming ? 0 : clampturret();
        double cur = getturretdeg();
        targetdeg = tgt; currentdeg = cur;
        double err = wrapangle(tgt - cur);
        turretontarget = Math.abs(err) < 2.0;

        double dt = turretDt.seconds();
        turretDt.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        double rawDesiredVel = wrapangle(tgt - prevTargetDeg) / dt;
        filteredDesiredTurretVel = filteredDesiredTurretVel*(1-turretVelAlpha) + rawDesiredVel*turretVelAlpha;

        if (Math.abs(err) <= tdead) { prevTargetDeg = tgt; lastTurretErr = err; turret.setPower(0); return; }

        double errDiff  = wrapangle(err - lastTurretErr);
        double feedback = tkp * err + tkd * (errDiff / dt);
        double staticFF = Math.abs(filteredDesiredTurretVel) > 1e-6 && Math.abs(err) > tffdead
                ? Math.copySign(tks, filteredDesiredTurretVel) : 0.0;
        double velFF    = tkv * filteredDesiredTurretVel;

        lastTurretErr = err; prevTargetDeg = tgt;
        turret.setPower(Math.max(-tmax, Math.min(tmax, feedback + staticFF + velFF)));
    }


    private void runflywheel() {

        flytarget  = targetvelocity;
        flycurrent = fly2.getVelocity();

        if (flytarget <= 1.0) { fly1.setPower(0); fly2.setPower(0); velontarget = false; return; }

        double vt    = vkVConst / Math.max(10.5, vsensor.getVoltage());
        double power = flycurrent < flytarget - vkBBThresh ? vt : vkF * flytarget * vt;
        power = Math.min(1.0, Math.max(0.0, power));

        fly1.setPower(power); fly2.setPower(power);
        velontarget = Math.abs(flytarget - flycurrent) < 40.0;
    }


    private double getturretdeg() { return cachedTurretDeg; }

    private double getdesiredturretdeg() {
        Pose p = follower.getPose();
        if (p == null) return getturretdeg();
        Pose vg = cachedVgoal != null ? cachedVgoal : new Pose(goalx, goaly, 0);
        double fieldangle = Math.atan2(vg.getY() - p.getY(), vg.getX() - p.getX());
        return wrapasym(Math.toDegrees(fieldangle - p.getHeading()) + 180 + turretoffset, thresh);
    }

    private double clampturret() {
        double d = wrapangle(getdesiredturretdeg());
        if (d >  thresh2) return  thresh2;
        if (d < -thresh)  return -thresh;
        return d;
    }

    private void updatelights() {
        if (blinkin == null) return;
        double pos = ballslocked ? 0.722 : (intakerev ? 0.277 : 0.0);
        pos = Math.round(pos * 1000.0) / 1000.0;
        if (Math.abs(lastlightpos - pos) > 0.001) { blinkin.setPosition(pos); lastlightpos = pos; }
    }

    private double wrapangle(double a)          { return ((a + 180) % 360 + 360) % 360 - 180; }
    private double wrapasym(double a, double n) { return ((a + n)   % 360 + 360) % 360 - n;   }

    private void sendtelem() {
        telemetry.addData("turret target",   targetdeg);
        telemetry.addData("turret current",  currentdeg);
        telemetry.addData("turret error",    wrapangle(targetdeg - currentdeg));
        telemetry.addData("turret ontarget", turretontarget);
        telemetry.addData("fly target",      flytarget);
        telemetry.addData("fly current",     flycurrent);
        telemetry.addData("fly error",       flytarget - flycurrent);
        telemetry.addData("fly ontarget",    velontarget);
        telemetry.addData("hood",            hoodbase);
        telemetry.addData("hold",            holdpos);
        telemetry.addData("ball state",      ballstate);
        telemetry.addData("slow shoot",      slowShoot2);
        telemetry.addData("slow step",       slowStep);
        telemetry.addData("sotm",            sotmActive);
        telemetry.addData("intake A",        String.format("%.2f", intake.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("d0",              d0.getVoltage());
        telemetry.addData("d1",              d1.getVoltage());
        telemetry.addData("d2",              d2.getVoltage());
        telemetry.addData("ll status",       llStatus);
        telemetry.addData("ll ta",           llTa);
        telemetry.addData("ll x",            llX);
        telemetry.addData("ll y",            llY);
        telemetry.addData("ll jump",         llJump);
        telemetry.addData("kpX",             kpX);
        telemetry.addData("turret vel",      filteredDesiredTurretVel);
        Pose p = follower.getPose();
        if (p != null) { telemetry.addData("x", p.getX()); telemetry.addData("y", p.getY()); }
    }

    @Override public void opLoopHook() {}
    @Override public void opTeardown() {}
}