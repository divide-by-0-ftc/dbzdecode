package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.auton.Tuning.draw;
import static org.firstinspires.ftc.teamcode.auton.Tuning.drawOnlyCurrent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

@Config
@TeleOp(name = "V2BLUE")
public class V2BLUE extends DbzOpMode {
    private enum TurretState { NORMAL, CENTERING }
    private enum BallState { IDLE, REVERSING, LOCKED }

    public static double targetx = 0, targety = 144;
    public static double shot1 = 300, shot2 = 600, shotret = 1000;
    public static double servooff = 0.035;
    public static double push0 = 0.85, push1 = 0.67, push2 = 0.47, push3 = 0.22;
    public static double lockpos = 0.71;
    public static double twitch = 0.76;


    public static double sticky = 0.15;

    public static double faroffset = 200;
    public static double holdopen = 0.8, holdclose = 0.467;
    public static double dthresh = 0.17, dthresh1 = 0.193, dthresh2 = 0.175;
    public static double dipamt = 0.0, dipdelay = 0.1, dipdur = 0.5;
    public static double vela = -0.0157003, velb = 11.6092, velc = 727.08688;
    public static double hooda = -0.0000876693, hoodb = 0.0228448, hoodc = -0.779915;
    public static double timea = 0.00002, timeb = 0.004, timec = 0.25;
    public static double goalx = 0, goaly = 144;
    public static double manualvel = 0, hooddefault = 0.5;
    public static double thresh = 220, thresh2 = 180;
    public static double tzero = 192;
    public static double tkp = 0.033, tki = 0.0, tkd = 0.0012;
    public static double tdead = 0.0, tmax = 1.0, tks = 0.015, tffdead = 0.0, toff = 2.0;
    public static double bangff = 1;
    public static double bang2ff = 1.8;

    public static boolean visionRelocalizeEnabled = true;
    public static double visionTaMin = 0.05;
    public static double visionMaxJump = 40.0;
    public static double visionCooldownSec = 1.5;
    public static double outlierThreshold = 4.0;
    public static double maxRelocalizeVelocity = 2.0;
    public static double maxRotVelocity = 1.0;
    public static double maxTurretVelocity = 5.0;
    public static double xoffset = 0.0;
    public static double yoffset = 6.0;

    public static double turretOffsetX = -35.0 * 0.03937;
    public static double turretOffsetY = 0.0;
    public static double camOffsetX = 158.37973 * 0.03937;
    public static double camOffsetY = 0.0;

    protected Servo rpush, lpush, hood, hold, blinkin;
    protected DcMotorEx intake, fly1, fly2, turret;
    private VoltageSensor vsensor;
    private AnalogInput tenc, d0, d1, d2;
    private PIDController tpid;
    private Limelight3A limelight;

    private final ElapsedTime intaketimer  = new ElapsedTime();
    private final ElapsedTime detecttimer  = new ElapsedTime();
    private final ElapsedTime diptimer     = new ElapsedTime();
    private final ElapsedTime holdtimer    = new ElapsedTime();
    private final ElapsedTime revtimer     = new ElapsedTime();
    private final ElapsedTime st0          = new ElapsedTime();
    private final ElapsedTime st1          = new ElapsedTime();
    private final ElapsedTime st2          = new ElapsedTime();
    private final ElapsedTime visionCooldown = new ElapsedTime();

    private TurretState turretstate = TurretState.NORMAL;
    private BallState   ballstate   = BallState.IDLE;

    private boolean shoot = false, lastshoot = false;
    private boolean fastmode = true;
    private boolean autohood = true, lasta = false;
    private boolean aiming = true, lastaim = false;
    private boolean intakefwd = false, intakerev = false;
    private boolean lastlb = false, lastrb = false;
    private boolean lastr1 = false, lastl1 = false;
    private boolean lastdpadup2 = false, lastdpaddn2 = false;
    private boolean latch0 = false, latch1 = false, latch2 = false;
    private boolean prevdetect = false;
    private boolean ballslocked = false;
    private boolean holdoverride = false;
    private boolean holdwait = false;
    private boolean dipping = false, dipdone = false;
    private boolean turretontarget = false, velontarget = false;

    private double holdposition  = holdclose;
    private double hoodbase      = hooddefault;
    private double targetvelocity = 0.0;
    private double turretoffset  = 0.0;
    private double lastlightpos  = -1;
    private double targetdeg = 0.0, currentdeg = 0.0;
    private double flytarget = 0.0, flycurrent = 0.0;

    private static final int QUEUE_SIZE = 3;
    private final double[] queuedX = new double[3];
    private final double[] queuedY = new double[3];
    private int queueIndex = 0;
    private boolean readInProgress = false;

    private double lastTurretAngleDeg = 0;
    private double lastHeadingRad = 0;
    private long lastVisionTimeMs = 0;
    private double turretVelocityDegS = 0;
    private double rotVelocityRadS = 0;

    private double llPedroX = 0.0;
    private double llPedroY = 0.0;
    private double llTa = 0.0;
    private double llPoseJump = 0.0;
    private boolean llValid = false;
    private boolean llApplied = false;
    private String llStatus = "IDLE";

    public static Follower follower;
    @IgnoreConfigurable
    public static TelemetryManager telemetrym;

    @Override
    public void opInit() {
        rpush = hardwareMap.get(Servo.class, "rightpushServo");
        lpush = hardwareMap.get(Servo.class, "leftpushServo");
        hood  = hardwareMap.get(Servo.class, "hoodServo");
        hold  = hardwareMap.get(Servo.class, "holdServo");
        blinkin = hardwareMap.get(Servo.class, "light");

        d0 = hardwareMap.get(AnalogInput.class, "distancez");
        d1 = hardwareMap.get(AnalogInput.class, "distance1");
        d2 = hardwareMap.get(AnalogInput.class, "distance2");

        hoodbase = hooddefault;
        hood.setPosition(hoodbase);
        holdposition = holdclose;
        hold.setPosition(holdposition);
        lpush.setPosition(push0);
        rpush.setPosition(push0 - servooff);

        intake = robot.intakeMotor;
        fly1   = robot.outtake1Motor;
        fly2   = robot.outtake2Motor;

        fly1.setDirection(DcMotorEx.Direction.REVERSE);
        fly2.setDirection(DcMotorEx.Direction.FORWARD);
        fly1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        telemetry = new com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetrym = PanelsTelemetry.INSTANCE.getTelemetry();

        tpid = new PIDController(tkp, tki, tkd);
        tpid.setTolerance(1.0);

        vsensor = hardwareMap.voltageSensor.iterator().next();

        turret = hardwareMap.get(DcMotorEx.class, DbzHardwareMap.Motor.turret.getName());
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);

        tenc = hardwareMap.get(AnalogInput.class, "turretEncoder");

        follower = ConstantsTele.createFollower(hardwareMap);
        follower.setStartingPose(org.firstinspires.ftc.teamcode.tele.PoseCache.lastPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        visionCooldown.reset();
        lastVisionTimeMs = System.currentTimeMillis();

        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.update();
        follower.update();

        if (follower.getCurrentPath() != null) drawOnlyCurrent();

        follower.startTeleopDrive();
    }

    @Override
    public void opLoop() {
        updatelights();

        boolean abtn = gamepad1.a;
        if (abtn && !lasta) autohood = !autohood;
        lasta = abtn;

        boolean dpadup2 = gamepad2.dpad_up;
        boolean dpaddn2 = gamepad2.dpad_down;
        if (dpadup2 && !lastdpadup2) { holdoverride = true; holdposition = holdopen; }
        if (dpaddn2 && !lastdpaddn2) { holdoverride = true; holdposition = holdclose; }
        lastdpadup2 = dpadup2;
        lastdpaddn2 = dpaddn2;

        boolean rb2 = gamepad1.dpad_right;
        boolean lb2 = gamepad1.dpad_left;
        if (rb2 && !lastr1) turretoffset -= toff;
        if (lb2 && !lastl1) turretoffset += toff;
        lastr1 = rb2;
        lastl1 = lb2;

        double mult = gamepad1.left_trigger > 0.1 ? 0.3 : 1.0;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * mult,
                -gamepad1.left_stick_x * mult,
                -gamepad1.right_stick_x * mult,
                true
        );
        follower.update();
        updateVisionRelocalization();

        regressions();
        dipshot();
        runballdetection();
        shootfast();
        activeintake();
        updatehold();

        Pose p = follower.getPose();
        if (p != null) {
            double dx = goalx - p.getX();
            double dy = goaly - p.getY();
            telemetrym.addData("dist", Math.hypot(dx, dy));
            if (Math.hypot(dx, dy) > 125) targetvelocity += faroffset;
        } else {
            telemetrym.addData("Pose", "NULL");
        }

        if (dbzGamepad1.x) { follower.setPose(new Pose(15, 111, Math.toRadians(90))); turretoffset = 0; }
        if (dbzGamepad1.y) { follower.setPose(new Pose(9.76378, 8.661, Math.toRadians(0))); turretoffset = 0; }

        aim();
        runflywheel();

        if (follower.getCurrentPath() != null) draw();

        sendtelem();
        telemetrym.update(telemetry);
        telemetry.update();
    }

    private void updateVisionRelocalization() {
        llValid   = false;
        llApplied = false;
        llTa      = 0.0;

        Pose current = follower.getPose();
        if (current == null || limelight == null) { llStatus = "NO_POSE"; return; }

        double robotHeadingRad = current.getHeading();
        double turretAngleRad  = Math.toRadians(getturretdeg());

        limelight.updateRobotOrientation(Math.toDegrees(robotHeadingRad + turretAngleRad) + 90.0);

        long now = System.currentTimeMillis();
        double dt = (now - lastVisionTimeMs) / 1000.0;
        if (dt > 0) {
            turretVelocityDegS = (getturretdeg() - lastTurretAngleDeg) / dt;
            rotVelocityRadS = (robotHeadingRad - lastHeadingRad) / dt;
        }
        lastTurretAngleDeg = getturretdeg();
        lastHeadingRad = robotHeadingRad;
        lastVisionTimeMs = now;

        com.pedropathing.math.Vector vel = follower.getVelocity();
        double driveVel = vel != null ? Math.hypot(vel.getXComponent(), vel.getYComponent()) : 0.0;

        boolean motionOk = Math.abs(driveVel) <= maxRelocalizeVelocity
                && Math.abs(rotVelocityRadS) <= maxRotVelocity
                && Math.abs(turretVelocityDegS) <= maxTurretVelocity;

        if (!motionOk) {
            readInProgress = false;
            queueIndex = 0;
            llStatus = "MOVING";
            return;
        }

        if (visionCooldown.seconds() < visionCooldownSec) {
            llStatus = "COOLDOWN";
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) { llStatus = "NO_RESULT"; return; }
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) { llStatus = "NO_TAG"; return; }

        llTa = result.getTa();
        if (llTa < visionTaMin) { llStatus = "LOW_TA"; return; }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) { llStatus = "NO_MT2"; return; }

        llValid = true;

        double llX = 72 - (botpose.getPosition().x * 39.3701);
        double llY = 72 + (botpose.getPosition().y * 39.3701);

        double thetaCamera = robotHeadingRad + turretAngleRad;
        double camFieldX = 0;
        double camFieldY = 0;
        double turretFieldX = 0;
        double turretFieldY = 0;
        double robotX = llX - camFieldX - turretFieldX;
        double robotY = llY - camFieldY - turretFieldY;

        double rawPedroX = robotY + xoffset;
        double rawPedroY = robotX + yoffset;

        if (!readInProgress) {
            if (visionCooldown.seconds() < visionCooldownSec) {
                llStatus = "COOLDOWN";
                return;
            }
            queueIndex = 0;
            readInProgress = true;
        }

        queuedX[queueIndex] = rawPedroX;
        queuedY[queueIndex] = rawPedroY;
        queueIndex++;

        if (queueIndex < QUEUE_SIZE) {
            llStatus = "COLLECTING " + queueIndex + "/" + QUEUE_SIZE;
            return;
        }

        readInProgress = false;
        queueIndex = 0;

        double d0 = hypot2d(queuedX[0] - midX(1,2), queuedY[0] - midY(1,2));
        double d1 = hypot2d(queuedX[1] - midX(0,2), queuedY[1] - midY(0,2));
        double d2 = hypot2d(queuedX[2] - midX(0,1), queuedY[2] - midY(0,1));

        if (Math.max(d0, Math.max(d1, d2)) > outlierThreshold) {
            llStatus = "OUTLIER";
            return;
        }

        llPedroX = (queuedX[0] + queuedX[1] + queuedX[2]) / 3.0;
        llPedroY = (queuedY[0] + queuedY[1] + queuedY[2]) / 3.0;

        llPoseJump = Math.hypot(llPedroX - current.getX(), llPedroY - current.getY());

        if (!visionRelocalizeEnabled) { llStatus = "DISABLED"; return; }
        if (llPoseJump > visionMaxJump) { llStatus = "JUMP_" + String.format("%.1f", llPoseJump); return; }

        follower.setPose(new Pose(llPedroX, llPedroY, current.getHeading()));
        llApplied = true;
        llStatus = "APPLIED";
        visionCooldown.reset();
    }

    private double midX(int a, int b) { return (queuedX[a] + queuedX[b]) / 2.0; }
    private double midY(int a, int b) { return (queuedY[a] + queuedY[b]) / 2.0; }
    private double hypot2d(double dx, double dy) { return Math.hypot(dx, dy); }

    private void updatelights() {
        if (blinkin == null) return;
        double pos = ballslocked ? 0.722 : (intakerev ? 0.277 : 0.0);
        pos = Math.round(pos * 1000.0) / 1000.0;
        if (Math.abs(lastlightpos - pos) > 0.001) {
            blinkin.setPosition(pos);
            lastlightpos = pos;
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
                        holdoverride = false;
                        holdposition = holdclose;
                        lpush.setPosition(lockpos);
                        rpush.setPosition(lockpos - servooff);
                        intake.setPower(-1);
                        revtimer.reset();
                        ballstate = BallState.REVERSING;
                        prevdetect = false;
                        ballslocked = true;
                    }
                } else if (!hit) {
                    prevdetect = false;
                    ballslocked = false;
                }
                break;

            case REVERSING:
                holdoverride = false;
                holdposition = holdclose;
                if (!shoot && revtimer.seconds() < 0.5) {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                    intake.setPower(-1);
                }
                if (revtimer.seconds() >= 0.5){
                    intake.setPower(0);
                    lpush.setPosition(twitch);
                    rpush.setPosition(twitch - servooff);
                }
                if(revtimer.seconds() >= 0.7){
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                }
                if (revtimer.seconds() >= 3.0) {
                    intake.setPower(0);
                    lpush.setPosition(push0);
                    rpush.setPosition(push0 - servooff);
                    ballstate = BallState.LOCKED;
                }
                break;

            case LOCKED:
                if (!shoot) { lpush.setPosition(lockpos); rpush.setPosition(lockpos - servooff); }
                if (shoot) intake.setPower(1);
                break;
        }
    }

    private void regressions() {
        Pose p = follower.getPose();
        if (p == null) return;
        double maxvel = 1900;
        if (autohood) {
            double dist = Math.hypot(goalx - p.getX(), goaly - p.getY());
            hoodbase      = Math.max(0.0, Math.min(1.0, hooda * dist * dist + hoodb * dist + hoodc));
            targetvelocity = Math.max(-maxvel, Math.min(maxvel, vela * dist * dist + velb * dist + velc));
        } else {
            hoodbase       = hooddefault;
            targetvelocity = manualvel;
        }
    }

    private void dipshot() {
        if (shoot && !dipping && !dipdone) { dipping = true; diptimer.reset(); }
        if (!shoot) { dipping = false; dipdone = false; hood.setPosition(hoodbase); return; }
        if (dipping) {
            double t = diptimer.seconds();
            if (t < dipdelay) {
                hood.setPosition(hoodbase);
            } else if (t < dipdelay + dipdur) {
                hood.setPosition(Math.max(0.0, hoodbase - dipamt));
            } else {
                hood.setPosition(hoodbase);
                dipping = false;
                dipdone = true;
            }
        }
    }

    private Pose virtualgoal(Pose p) {
        return new Pose(goalx, goaly, 0);
    }

    private void shootfast() {
        boolean trig = dbzGamepad1.right_trigger > 0.1;

        if (!fastmode) {
            if (trig && !lastshoot && !shoot) {
                boolean had3 = ballslocked;
                boolean holdready = hold.getPosition() >= holdopen - 0.01;
                holdoverride = false; holdposition = holdclose;
                intaketimer.reset(); shoot = true; ballslocked = false; dipping = false; dipdone = false;
                if (had3 && !holdready) { holdwait = true; holdtimer.reset(); }
                else { holdwait = false; lpush.setPosition(push1); rpush.setPosition(push1 - servooff); intaketimer.reset(); }
            }
            if (shoot && holdwait) {
                if (holdtimer.milliseconds() >= 200) {
                    holdwait = false; lpush.setPosition(push1); rpush.setPosition(push1 - servooff); intaketimer.reset();
                }
            } else if (shoot) {
                if (intaketimer.milliseconds() > shotret) {
                    lpush.setPosition(push0); rpush.setPosition(push0 - servooff);
                    holdposition = holdclose; shoot = false; resetshot();
                } else if (intaketimer.milliseconds() > shot2) {
                    lpush.setPosition(push3); rpush.setPosition(push3 - servooff);
                } else if (intaketimer.milliseconds() > shot1) {
                    lpush.setPosition(push2); rpush.setPosition(push2 - servooff);
                }
            }
        } else {
            if (trig && !lastshoot && !shoot) {
                boolean had3 = ballslocked;
                boolean holdready = hold.getPosition() >= holdopen - 0.01;
                ballslocked = false; holdoverride = false; holdposition = holdopen;
                intaketimer.reset(); shoot = true; dipping = false; dipdone = false;
                if (had3 && !holdready) { holdwait = true; holdtimer.reset(); }
                else { holdwait = false; lpush.setPosition(push3); rpush.setPosition(push3 - servooff); intaketimer.reset(); }
            }
            if (shoot && holdwait) {
                if (holdtimer.milliseconds() >= 200) {
                    holdwait = false; lpush.setPosition(push3); rpush.setPosition(push3 - servooff); intaketimer.reset();
                }
            } else if (shoot && intaketimer.milliseconds() > 700) {
                lpush.setPosition(push0); rpush.setPosition(push0 - servooff);
                holdposition = holdclose; shoot = false; resetshot();
            }
            if (!shoot) intaketimer.reset();
        }

        lastshoot = trig;
        telemetry.addData("fastmode", fastmode);
        telemetry.addData("holdwait", holdwait);
    }

    private void resetshot() {
        ballslocked = false; ballstate = BallState.IDLE;
        holdoverride = false; holdposition = holdclose; holdwait = false;
        lpush.setPosition(push0); rpush.setPosition(push0 - servooff);
        intakefwd = true; intakerev = false; intake.setPower(1);
    }

    private void activeintake() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        if (shoot)      { intake.setPower(0);  lastrb = rb; lastlb = lb; return; }
        if (ballslocked){ intake.setPower(-1); lastrb = rb; lastlb = lb; return; }

        if (rb && !lastrb) { intakefwd = !intakefwd; intakerev = false; }
        if (lb && !lastlb) { intakerev = !intakerev; intakefwd = false; }

        intake.setPower(intakefwd ? 1 : intakerev ? -1 : 0);

        if (!ballslocked && !shoot) { lpush.setPosition(push0); rpush.setPosition(push0 - servooff); }
        lastrb = rb; lastlb = lb;
    }

    private void updatehold() {
        if (!holdoverride) {
            if (shoot) {
                holdposition = holdopen;
            } else if (ballstate == BallState.REVERSING) {
                holdposition = revtimer.seconds() >= 0.5 ? holdopen : holdclose;
            } else if (ballstate == BallState.LOCKED) {
                holdposition = holdopen;
            }
        }
        hold.setPosition(holdposition);
    }

    private void aim() {
        boolean aimBtn = gamepad1.dpad_up;
        if (aimBtn && !lastaim) { aiming = !aiming; tpid.reset(); }
        lastaim = aimBtn;

        double tgtangle;
        if (!aiming) {
            turretstate = TurretState.NORMAL;
            tgtangle = 0.0;
        } else {
            double clamped = clampturret();
            switch (turretstate) {
                case NORMAL:
                    if (Math.abs(clamped - getturretdeg()) <= thresh) { tgtangle = clamped; }
                    else { turretstate = TurretState.CENTERING; tgtangle = 0.0; }
                    break;
                case CENTERING:
                    tgtangle = 0.0;
                    if (Math.abs(getturretdeg()) < 5.0) turretstate = TurretState.NORMAL;
                    break;
                default:
                    tgtangle = 0.0; turretstate = TurretState.NORMAL; break;
            }
        }

        double cur = getturretdeg();
        targetdeg = tgtangle; currentdeg = cur;
        double err = wrapangle(tgtangle - cur);
        turretontarget = Math.abs(err) < 2.0;

        if (Math.abs(err) <= tdead) { turret.setPower(0); return; }

        tpid.setPID(tkp, tki, tkd);
        double out = tpid.calculate(cur, tgtangle);
        double ff  = Math.abs(err) > tffdead ? Math.copySign(tks, err) : 0.0;
        turret.setPower(Math.max(-tmax, Math.min(tmax, out + ff)));
    }

    private void runflywheel() {
        flytarget  = targetvelocity;
        flycurrent = fly2.getVelocity();

        if (Math.abs(targetvelocity) <= 1.0) {
            fly1.setPower(0); fly2.setPower(0); velontarget = false; return;
        }

        double maxvel = fly2.getMotorType().getMaxRPM() * fly2.getMotorType().getTicksPerRev() / 60.0;
        double batv   = Math.max(10.5, vsensor.getVoltage());
        double ff     = bangff * (targetvelocity / maxvel) * (12.0 / batv);
        double bb     = flycurrent < targetvelocity ? 1.0 : 0.0;
        double power  = Math.min(1.0, bb + ff);

        fly1.setPower(power); fly2.setPower(power);
        velontarget = Math.abs(targetvelocity - flycurrent) < 40.0;
    }

    private double getturretdeg() {
        double angle = (tenc.getVoltage() / tenc.getMaxVoltage()) * 360.0 - tzero;
        return wrapasym(angle, thresh);
    }

    private double getdesiredturretdeg() {
        Pose p = follower.getPose();
        if (p == null) return getturretdeg();
        Pose vgoal = virtualgoal(p);
        double fieldangle = Math.atan2(vgoal.getY() - p.getY(), vgoal.getX() - p.getX());
        return wrapasym(Math.toDegrees(fieldangle - p.getHeading()) + turretoffset, thresh);
    }

    private double clampturret() {
        double d = wrapasym(getdesiredturretdeg(), thresh);
        if (d > thresh2) return thresh2;
        if (d < -thresh) return -thresh;
        return d;
    }

    private double wrapangle(double a)         { return ((a + 180) % 360 + 360) % 360 - 180; }
    private double wrapasym(double a, double n){ return ((a + n)   % 360 + 360) % 360 - n; }

    private void sendtelem() {
        telemetry.addData("target degrees", targetdeg);
        telemetry.addData("current degrees", currentdeg);
        telemetry.addData("turret error", wrapangle(targetdeg - currentdeg));
        telemetry.addData("turret on target", turretontarget);
        telemetry.addData("intake current", String.format("%.2f", intake.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("flywheel target", flytarget);
        telemetry.addData("flywheel current", flycurrent);
        telemetry.addData("flywheel error", flytarget - flycurrent);
        telemetry.addData("on target", velontarget);
        telemetry.addData("hold pos", holdposition);
        telemetry.addData("hold override", holdoverride);
        telemetry.addData("hold state", ballstate);

        telemetry.addData("d0", d0.getVoltage());
        telemetry.addData("d1", d1.getVoltage());
        telemetry.addData("d2", d2.getVoltage());


        telemetry.addData("ll status", llStatus);
        telemetry.addData("ll valid", llValid);
        telemetry.addData("ll applied", llApplied);
        telemetry.addData("ll ta", llTa);
        telemetry.addData("ll pedro x", llPedroX);
        telemetry.addData("ll pedro y", llPedroY);
        telemetry.addData("ll pose jump", llPoseJump);
        telemetry.addData("ll cooldown", String.format("%.2f", visionCooldown.seconds()));
        telemetry.addData("ll turret vel deg/s", String.format("%.2f", turretVelocityDegS));
        telemetry.addData("ll rot vel rad/s", String.format("%.3f", rotVelocityRadS));
    }

    @Override public void opLoopHook() {}
    @Override public void opTeardown() {}
}