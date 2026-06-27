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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auton.ConstantsTele;
import org.firstinspires.ftc.teamcode.extensions.DbzHardwareMap;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

import java.util.List;

@Config
@TeleOp(name = "REDV5")
public class REDV5 extends DbzOpMode {

    private enum BallState { IDLE, REVERSING, LOCKED }

    public static double push0 = 0.85, push1 = 0.67, push2 = 0.47, push3 = 0.185;
    public static double lockpos = 0.7275, twitch = 0.767, servooff = 0.035;
    public static double holdopen = 0.8, holdclose = 0.467;
    public static double hooddefault = 0.3;
    public static double dipamt = 0.1, dipdelay = 0.15, dipdur = 0.5;

    public static double dthresh = 0.166, dthresh1 = 0.167, dthresh2 = 0.166 ;
    public static double sticky = 0.1;

    public static double[] lutD = {55, 65.2, 75.2, 85.2, 100};
    public static double[] lutH = {0.27, 0.3, 0.40, 0.44, 0.53};
    public static double[] lutV = {1360, 1430, 1460, 1585, 1715};

    public static double[] lutD2 = {57.26, 69.89, 79.32, 90.50};

    public static double[] lutTof = {0.67,0.7, 0.75, 0.77};

    private double tofbase = 0;
    public static double manualvel = 0;
    public static double timea = 0.00002, timeb = 0.004, timec = 0.25;

    public static double goalx = 144, goaly = 144;

    // Turret controller
    public static double tkp = 0.03, tkd = 0.0015, tkv = 0.0016, tks = 0.0,
            tffdead = 0.0;
    public static double tdead = 0.0, tmax = 1.0, toff = 2.0;
    public static double thresh = 168, thresh2 = 158, tzero = 180;
    public static double turretVelAlpha = 0.2;

    // Turret SOTM time-of-flight (seconds) — replace with a real model later
    public static double turretTof = 0.5;

    // Placeholder shooting zones (CCW winding) — fill in real field coordinates
    public static double[][] shootZone1 = {{0, 144 - 6}, {72, 72- 6}, {141, 144- 6}};;
    public static double[][] shootZone2 = {{49- 6, 10},  {72, 33 + 6}, {95 + 6, 10}};

    public static double vkF = 0.00038, vkBBThresh = 50.0, vkVConst = 12.0;

    public static double sotmDelay = 0.1, sotmMinVel = 1.5, sotmScale = 1;
    public static double sotmVelAlpha = 0.3;

    public static double veloff = 10;

    public static double brakeWait = 0.3, brakeShootDelay = 0.0;

    public static double arch = -0.09, arcv = -39.5, sotg = 1;
    public static int voltagePollEvery = 15;

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
    private final ElapsedTime turretDt    = new ElapsedTime();
    private final ElapsedTime loopTimer   = new ElapsedTime();

    private BallState ballstate = BallState.IDLE;

    private boolean shoot         = false, lastshoot    = false, hasShot
            = false;
    private boolean autohood      = true,  lasta        = false;
    private boolean aiming        = true,  lastaim      = false;
    private boolean sotmActive    = false, lastsotm     = false;
    private boolean intakefwd     = true,  intakerev    = false;
    private boolean lastlb        = false, lastrb       = false;
    private boolean lastlb2       = false;
    private boolean lastr1        = false, lastl1       = false;
    private boolean lastrup       = false, lastldown    = false;
    private boolean lastB         = false;
    private boolean latch0        = false, latch1       = false, latch2  = false;
    private boolean prevdetect    = false, ballslocked  = false;
    private boolean holdoverride  = false, holdwait     = false;
    private boolean dipping       = false, dipdone      = false;
    private boolean turretontarget= false, velontarget  = false;
    private boolean braking       = false;
    private boolean lastBrakeState= false;
    private boolean slowShoot2    = false;

    private int slowStep = 0;
    private final ElapsedTime slowShotTimer = new ElapsedTime();
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

    private double cachedVoltage = 12.0;
    private double lastLoopMs    = 0;

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

        turret = hardwareMap.get(DcMotorEx.class,
                DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
        tenc = hardwareMap.get(AnalogInput.class, "turretEncoder");

        vsensor = hardwareMap.voltageSensor.iterator().next();
        cachedVoltage = vsensor.getVoltage();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs)
            h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        telemetry = new
                com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetrym = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = ConstantsTele.createFollower(hardwareMap);
        follower.setStartingPose(PoseCache.lastPose);

        turretDt.reset();
        loopTimer.reset();

        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.update();
        follower.update();
        if (follower.getCurrentPath() != null) drawOnlyCurrent();
        follower.startTeleopDrive();
    }


    @Override
    public void opLoop() {

        lastLoopMs = loopTimer.milliseconds();
        loopTimer.reset();

        for (LynxModule h : hubs) h.clearBulkCache();

        if (loopCount % voltagePollEvery == 0) cachedVoltage =
                vsensor.getVoltage();

        cachedTurretDeg = wrapangle((tenc.getVoltage() / tenc.getMaxVoltage())
                * 360.0 - tzero);

        Pose robotPose  = follower.getPose();
        cachedVgoal     = (robotPose != null) ? virtualgoal(robotPose) : new
                Pose(goalx, goaly, 0);

        boolean abtn = gamepad1.a;
        if (abtn && !lasta) autohood = !autohood;
        lasta = abtn;

        if (gamepad2.dpad_right && !lastr1) turretoffset -= toff;
        if (gamepad2.dpad_left  && !lastl1) turretoffset += toff;
        lastr1 = gamepad2.dpad_right;
        lastl1 = gamepad2.dpad_left;

        if (gamepad2.dpad_down && !lastldown) arcv -= veloff;
        if (gamepad2.dpad_up   && !lastrup)   arcv += veloff;
        lastrup   = gamepad2.dpad_up;
        lastldown = gamepad2.dpad_down;

        boolean b = gamepad2.b;
        if (b && !lastB) {
            slowShoot2 = !slowShoot2; slowStep = 0; slowReturning = false;
            if (!slowShoot2) { braking = false; follower.startTeleopDrive(); }
        }
        lastB = b;

        if (slowShoot2) {
            boolean s1 = gamepad2.left_bumper;
            if (!slowReturning && s1 && !lastlb2) {
                slowStep++;
                if (slowStep == 1) {
                    lpush.setPosition(push1); rpush.setPosition(push1 -
                            servooff);
                } else if (slowStep == 2) {
                    lpush.setPosition(push2); rpush.setPosition(push2 -
                            servooff);
                } else {
                    lpush.setPosition(push3); rpush.setPosition(push3 -
                            servooff);
                    slowShotTimer.reset(); slowReturning = true;
                    ballslocked = false; ballstate = BallState.IDLE;
                }
            }
            if (slowReturning && slowShotTimer.milliseconds() > 500) {
                lpush.setPosition(push0); rpush.setPosition(push0 - servooff);
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
        if (slowShoot2) { if (braking) follower.startTeleopDrive(); braking =
                false; hasShot = false; lastBrakeState = false; }

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

        if (dbzGamepad1.x) { follower.setPose(new Pose(144-15.91160220994475,
                78.76243093922652, Math.toRadians(0))); turretoffset = 0; }
        if (dbzGamepad1.y) { follower.setPose(new Pose(144 - 9.76378, 8.661,
                Math.toRadians(180)));                 turretoffset = 0; }

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
            telemetrym.addData("dist", Math.hypot(goalx - robotPose.getX(),
                    goaly - robotPose.getY()));
        }

        boolean telemTick = (loopCount % 3 == 0);
        if (telemTick) {
            if (follower.getCurrentPath() != null) draw();
            sendtelem();
            telemetrym.update(telemetry);
            telemetry.update();
        }
        loopCount++;
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
                    if (!prevdetect) { detecttimer.reset(); prevdetect = true;
                    }
                    if (detecttimer.seconds() >= 0.2) {
                        latch0 = latch1 = latch2 = false;
                        holdoverride = false; holdpos = holdclose;
                        lpush.setPosition(lockpos); rpush.setPosition(lockpos
                                - servooff);
                        intake.setPower(-1); revtimer.reset();
                        ballstate = BallState.REVERSING; prevdetect = false;
                        ballslocked = true;
                    }
                } else if (!hit) {
                    prevdetect = false; ballslocked = false;
                }
                break;

            case REVERSING:
                holdoverride = false; holdpos = holdclose;
                if (!shoot && !slowShoot2 && revtimer.seconds() < 0.5) {
                    lpush.setPosition(lockpos); rpush.setPosition(lockpos -
                            servooff); intake.setPower(-1);
                }
                if (revtimer.seconds() >= 0.5 && !shoot && !slowShoot2) {
                    intake.setPower(1); lpush.setPosition(twitch);
                    rpush.setPosition(twitch - servooff);
                }
                if (revtimer.seconds() >= 0.85 && !shoot && !slowShoot2) {
                    intake.setPower(1); lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                    ballstate = BallState.LOCKED;}
                break;

            case LOCKED:
                if (!shoot && !slowShoot2) { lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff); }
                if (shoot) intake.setPower(1);
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

        Pose p = follower.getPose();
        double dist = Math.hypot(cachedVgoal.getX() - p.getX(), cachedVgoal.getY() - p.getY());

        hoodbase = Math.max(0.0, lerp(lutD, lutH, dist)) - arch;
        hoodbase = Math.min(hoodbase, 0.55);
        targetvelocity = Math.max(1250, lerp(lutD, lutV, dist)) - arcv;
        targetvelocity = Math.min(targetvelocity, 1700);
        tofbase = 0;

    }


    private void dipshot() {
        if (shoot && !dipping && !dipdone) { dipping = true; diptimer.reset();
        }
        if (!shoot) { dipping = false; dipdone = false;
            hood.setPosition(hoodbase); return; }
        if (dipping) {
            double t = diptimer.seconds();
            if      (t < dipdelay)          hood.setPosition(hoodbase);
            else if (t < dipdelay + dipdur) hood.setPosition(Math.max(0.0,
                    hoodbase - dipamt));
            else { hood.setPosition(hoodbase); dipping = false; dipdone =
                    true; }
        }
    }


    private Pose virtualgoal(Pose p) {

        com.pedropathing.math.Vector vel = follower.getVelocity();
        double rawVx = vel != null ? vel.getXComponent() : 0;
        double rawVy = vel != null ? vel.getYComponent() : 0;

        svx = svx * (1 - sotmVelAlpha) + rawVx * sotmVelAlpha;
        svy = svy * (1 - sotmVelAlpha) + rawVy * sotmVelAlpha;

        if (!sotmActive || Math.hypot(svx, svy) < sotmMinVel) return new
                Pose(goalx, goaly, 0);

        double dist1  = Math.hypot(goalx - p.getX(), goaly - p.getY());
        double tTotal = timea * dist1 * dist1 + timeb * dist1 + timec +
                sotmDelay;
        double vgX    = goalx - svx * tTotal * sotmScale;
        double vgY    = goaly - svy * tTotal * sotmScale;

        double dist2 = Math.hypot(vgX - p.getX(), vgY - p.getY());
        tTotal = timea * dist2 * dist2 + timeb * dist2 + timec + sotmDelay;
        vgX    = goalx - svx * tTotal * sotmScale;
        vgY    = goaly - svy * tTotal * sotmScale;

        return new Pose(vgX, vgY, 0);
    }


    private void fireShot() {
        if (slowShoot2) return;

        boolean had3      = ballslocked;
        boolean holdready = hold.getPosition() >= holdopen - 0.01;
        ballslocked = false; holdoverride = false;
        intaketimer.reset(); shoot = true; dipping = false; dipdone = false;

        holdpos = holdopen;
        if (had3 && !holdready) { holdwait = true; holdtimer.reset(); }
        else { holdwait = false; lpush.setPosition(push3);
            rpush.setPosition(push3 - servooff); intaketimer.reset(); }
    }

    private void shootfast() {
        if (slowShoot2) return;

        boolean trig = dbzGamepad1.right_trigger > 0.1;

        if (braking) {
            if (!hasShot && braketimer.seconds() >= brakeShootDelay) {
                fireShot(); hasShot = true; }
            if (braketimer.seconds() >= brakeWait) { braking = false; hasShot
                    = false; follower.startTeleopDrive(); }
        }

        if (shoot && holdwait && holdtimer.milliseconds() >= 200) {
            holdwait = false; lpush.setPosition(push3);
            rpush.setPosition(push3 - servooff); intaketimer.reset();
        } else if (shoot && !holdwait && intaketimer.milliseconds() > 500) {
            lpush.setPosition(push0); rpush.setPosition(push0 - servooff);
            holdpos = holdclose; shoot = false; resetshot();
        }
        if (!shoot && !braking) intaketimer.reset();

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

        if (shoot)                            { intake.setPower(0); lastrb =
                rb; lastlb = lb; return; }
        if (ballstate == BallState.REVERSING) { lastrb = rb; lastlb = lb;
            return; }

        if (rb && !lastrb) { intakefwd = !intakefwd; intakerev = false; }
        if (lb && !lastlb) { intakerev = !intakerev; intakefwd = false; }

        if (intakerev) {
            intake.setPower(-1);
            if (!slowShoot2) { lpush.setPosition(push0);
                rpush.setPosition(push0 - servooff); }
            ballstate = BallState.IDLE; ballslocked = false;
            lastrb = rb; lastlb = lb; return;
        }

        if (ballslocked) { intake.setPower(-1); lastrb = rb; lastlb = lb;
            return; }

        intake.setPower(intakefwd ? 1 : 0);
        if (!ballslocked && !shoot && !slowShoot2) { lpush.setPosition(push0);
            rpush.setPosition(push0 - servooff); }
        lastrb = rb; lastlb = lb;
    }

    private void updatehold() {
        if (!holdoverride) {
            if      (shoot)                            holdpos = holdopen;
            else if (ballstate == BallState.REVERSING) holdpos =
                    revtimer.seconds() >= 0.5 ? holdopen : holdclose;
            else if (ballstate == BallState.LOCKED)    holdpos = holdopen;
        }
        hold.setPosition(holdpos);
    }


    private void aim() {

        boolean aimBtn = gamepad1.dpad_up;
        if (aimBtn && !lastaim) {
            aiming = !aiming;
            lastTurretErr = 0; prevTargetDeg = getturretdeg();
            filteredDesiredTurretVel = 0; turretDt.reset();
        }
        lastaim = aimBtn;

        double tgt = !aiming ? 0 : clampturret();

        double cur = getturretdeg();
        targetdeg = tgt; currentdeg = cur;
        double err = tgt - cur;
        turretontarget = Math.abs(err) < 2.0;

        double dt = turretDt.seconds();
        turretDt.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        double rawDesiredVel = (tgt - prevTargetDeg) / dt;
        filteredDesiredTurretVel = filteredDesiredTurretVel * (1 -
                turretVelAlpha) + rawDesiredVel * turretVelAlpha;

        if (Math.abs(err) <= tdead) { prevTargetDeg = tgt; lastTurretErr =
                err; turret.setPower(0); return; }

        double errDiff  = err - lastTurretErr;
        double feedback = tkp * err + tkd * (errDiff / dt);
        double staticFF = Math.abs(filteredDesiredTurretVel) > 1e-6 &&
                Math.abs(err) > tffdead
                ? Math.copySign(tks, filteredDesiredTurretVel) : 0.0;
        double velFF    = tkv * filteredDesiredTurretVel;

        lastTurretErr = err; prevTargetDeg = tgt;
        turret.setPower(Math.max(-tmax, Math.min(tmax, feedback + staticFF +
                velFF)));
    }


    private void runflywheel() {

        flytarget  = targetvelocity;
        flycurrent = fly2.getVelocity();

        if (flytarget <= 1.0) { fly1.setPower(0); fly2.setPower(0);
            velontarget = false; return; }

        double vt    = vkVConst / Math.max(10.5, cachedVoltage);
        double power = flycurrent < flytarget - vkBBThresh ? vt : vkF *
                flytarget * vt;
        power = Math.min(1.0, Math.max(0.0, power));

        fly1.setPower(power); fly2.setPower(power);
        velontarget = Math.abs(flytarget - flycurrent) < 40.0;
    }


    private double getturretdeg() { return cachedTurretDeg; }

    private double getdesiredturretdeg() {
        Pose p = follower.getPose();
        if (p == null) return getturretdeg();
        Pose vg = cachedVgoal != null ? cachedVgoal : new Pose(goalx, goaly, 0);

        com.pedropathing.math.Vector vel = follower.getVelocity();
        double vx = vel != null ? vel.getXComponent() : 0;
        double vy = vel != null ? vel.getYComponent() : 0;
        double aimX = vg.getX() - vx * tofbase;
        double aimY = vg.getY() - vy * tofbase;

        double fieldangle = Math.atan2(aimY - p.getY(), aimX - p.getX());
        return wrapangle(Math.toDegrees(fieldangle - p.getHeading()) + 180 + turretoffset);
    }

    private double clampturret() {
        double d = getdesiredturretdeg();
        if (d >  thresh2) return  thresh2;
        if (d < -thresh)  return -thresh;
        return d;
    }

    private static double[] closestPointOnSegment(double ax, double ay,
                                                  double bx, double by,
                                                  double px, double py) {
        double abx = bx - ax, aby = by - ay;
        double len2 = abx * abx + aby * aby;
        if (len2 < 1e-12) return new double[]{ax, ay};
        double t = Math.max(0, Math.min(1, ((px - ax) * abx + (py - ay) * aby)
                / len2));
        return new double[]{ax + t * abx, ay + t * aby};
    }

    private static double[] closestPointOnTriangle(double[][] tri, double px,
                                                   double py) {
        boolean inside = true;
        for (int i = 0; i < 3; i++) {
            double ax = tri[i][0],           ay = tri[i][1];
            double bx = tri[(i + 1) % 3][0], by = tri[(i + 1) % 3][1];
            if ((bx - ax) * (py - ay) - (by - ay) * (px - ax) < 0) { inside =
                    false; break; }
        }
        if (inside) return new double[]{px, py};
        double bestDist = Double.MAX_VALUE;
        double[] best = null;
        for (int i = 0; i < 3; i++) {
            double ax = tri[i][0],           ay = tri[i][1];
            double bx = tri[(i + 1) % 3][0], by = tri[(i + 1) % 3][1];
            double[] c = closestPointOnSegment(ax, ay, bx, by, px, py);
            double d = (c[0] - px) * (c[0] - px) + (c[1] - py) * (c[1] - py);
            if (d < bestDist) { bestDist = d; best = c; }
        }
        return best;
    }

    private double[] closestShootPoint(double px, double py) {
        double[] c1 = closestPointOnTriangle(shootZone1, px, py);
        double[] c2 = closestPointOnTriangle(shootZone2, px, py);
        double d1 = (c1[0] - px) * (c1[0] - px) + (c1[1] - py) * (c1[1] - py);
        double d2 = (c2[0] - px) * (c2[0] - px) + (c2[1] - py) * (c2[1] - py);
        return d1 <= d2 ? c1 : c2;
    }

    private void updatelights() {
        if (blinkin == null) return;
        double pos = ballslocked ? 0.722 : (intakerev ? 0.277 : 0.0);
        pos = Math.round(pos * 1000.0) / 1000.0;
        if (Math.abs(lastlightpos - pos) > 0.001) { blinkin.setPosition(pos);
            lastlightpos = pos; }
    }

    private double wrapangle(double a) { return ((a + 180) % 360 + 360) % 360
            - 180; }

    private void sendtelem() {
        telemetry.addData("loop ms",         String.format("%.1f",
                lastLoopMs));
        telemetry.addData("voltage",         tenc.getVoltage());
        telemetry.addData("turret target",   targetdeg);
        telemetry.addData("turret current",  currentdeg);
        telemetry.addData("turret error",    targetdeg - currentdeg);
        telemetry.addData("turret ontarget", turretontarget);
        telemetry.addData("turret desired",  getdesiredturretdeg());
        telemetry.addData("turret vel",      filteredDesiredTurretVel);
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
        telemetry.addData("intake A",        String.format("%.2f",
                intake.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("d0",              d0.getVoltage());
        telemetry.addData("d1",              d1.getVoltage());
        telemetry.addData("d2",              d2.getVoltage());

        Pose p = follower.getPose();
        if (p != null) { telemetry.addData("x", p.getX());
            telemetry.addData("y", p.getY()); }
    }

    @Override public void opLoopHook() {}
    @Override public void opTeardown() {}
}