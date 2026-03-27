package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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

@Config
@TeleOp(name = "V2RED")
public class V2RED extends DbzOpMode
{
    private enum TurretState { NORMAL, CENTERING }
    private enum BallState { IDLE, REVERSING, LOCKED }

    public static double targetx = 144, targety = 144;
    public static double shot1 = 300, shot2 = 600, shotret = 1000;
    public static double servooff = 0.035;
    public static double push0 = 0.85, push1 = 0.67, push2 = 0.47, push3 = 0.22;
    public static double lockpos = 0.71;
    public static double sticky = 0.15;
    public static double holdopen = 0.8, holdclose = 0.467;
    public static double dthresh = 0.157, dthresh1 = 0.173, dthresh2 = 0.155;
    public static double dipamt = 0.0, dipdelay = 0.5, dipdur = 0.15;
    public static double vela = -0.0157003, velb = 11.6092, velc = 727.08688;
    public static double hooda = -0.0000876693, hoodb = 0.0228448, hoodc = -0.779915;
    public static double timea = 0.00002, timeb = 0.004, timec = 0.25;
    public static double goalx = 144, goaly = 144;
    public static double manualvel = 0, hooddefault = 0.5;
    public static double thresh = 220, thresh2 = 180;
    public static double tzero = 181;
    public static double tkp = 0.02, tki = 0.0, tkd = 0.001;
    public static double tdead = 0.0, tmax = 1.0, tks = 0.01, tffdead = 0.0, toff = 2.0;
    public static double bangff = 0.85;
    public static double driftcorrx = 0.0, driftcorry = 0.0, driftcorrh = 0.0;

    protected Servo rpush, lpush, hood, hold, blinkin;
    protected DcMotorEx intake, fly1, fly2, turret;
    private VoltageSensor vsensor;
    private AnalogInput tenc, d0, d1, d2;
    private PIDController tpid;

    private ElapsedTime intaketimer = new ElapsedTime();
    private ElapsedTime detecttimer = new ElapsedTime();
    private ElapsedTime diptimer = new ElapsedTime();
    private ElapsedTime holdtimer = new ElapsedTime();
    private ElapsedTime revtimer = new ElapsedTime();
    private ElapsedTime st0 = new ElapsedTime();
    private ElapsedTime st1 = new ElapsedTime();
    private ElapsedTime st2 = new ElapsedTime();

    private TurretState turretstate = TurretState.NORMAL;
    private BallState ballstate = BallState.IDLE;

    private boolean shoot = false, lastshoot = false;
    private boolean fastmode = true, lastfast = false;
    private boolean autohood = true, lasta = false;
    private boolean aiming = true, lastaim = false;
    private boolean intakefwd = false, intakerev = false;
    private boolean lastlb = false, lastrb = false;
    private boolean lastr1 = false, lastl1 = false;
    private boolean lastdpadup1 = false, lastdpaddn1 = false;
    private boolean lastdpadup2 = false, lastdpaddn2 = false;
    private boolean latch0 = false, latch1 = false, latch2 = false;
    private boolean prevdetect = false;
    private boolean ballslocked = false;
    private boolean holdoverride = false;
    private boolean holdwait = false;
    private boolean dipping = false, dipdone = false;
    private boolean rpmmode = false;
    private boolean turretontarget = false, velontarget = false;

    private double holdposition = holdopen;
    private double hoodbase = hooddefault;
    private double targetvelocity = 0.0;
    private double turretoffset = 0.0;
    private double lastlightpos = -1;
    private double maxvelocity = 1900;
    private double targetdeg = 0.0, currentdeg = 0.0;
    private double flytarget = 0.0, flycurrent = 0.0;

    public static Follower follower;
    @IgnoreConfigurable
    public static TelemetryManager telemetrym;

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

        hoodbase = hooddefault;
        hood.setPosition(hoodbase);
        holdposition = holdopen;
        hold.setPosition(holdposition);
        lpush.setPosition(push0);
        rpush.setPosition(push0 - servooff);

        intake = robot.intakeMotor;
        fly1 = robot.outtake1Motor;
        fly2 = robot.outtake2Motor;

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
        follower.setStartingPose(PoseCache.lastPose);


        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.update();
        follower.update();
        follower.startTeleopDrive();
    }

    @Override
    public void opLoop()
    {
        updatelights();

        boolean ftoggle = gamepad1.left_stick_button;
        if (ftoggle && !lastfast) fastmode = !fastmode;
        lastfast = ftoggle;

        boolean abtn = gamepad1.a;
        if (abtn && !lasta) autohood = !autohood;
        lasta = abtn;

//        boolean dpadup1 = gamepad1.dpad_up, dpaddn1 = gamepad1.dpad_down;
//        if (dpadup1 && !lastdpadup1)
//        {
//            rpmmode = true;
//            maxvelocity = fly2.getMotorType().getMaxRPM() * fly2.getMotorType().getTicksPerRev() / 60.0;
//        }
//        if (dpaddn1 && !lastdpaddn1) { rpmmode = false; maxvelocity = 1900; }
//        lastdpadup1 = dpadup1;
//        lastdpaddn1 = dpaddn1;

        boolean dpadup2 = gamepad2.dpad_up, dpaddn2 = gamepad2.dpad_down;
        if (dpadup2 && !lastdpadup2) { holdoverride = true; holdposition = holdopen; }
        if (dpaddn2 && !lastdpaddn2) { holdoverride = true; holdposition = holdclose; }
        lastdpadup2 = dpadup2;
        lastdpaddn2 = dpaddn2;

        boolean rb2 = gamepad2.right_bumper, lb2 = gamepad2.left_bumper;
        if (rb2 && !lastr1) turretoffset -= toff;
        if (lb2 && !lastl1) turretoffset += toff;
        lastr1 = rb2;
        lastl1 = lb2;

        double mult = gamepad1.left_trigger > 0.1 ? 0.3 : 1.0;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * mult,
                -gamepad1.left_stick_x * mult,
                -gamepad1.right_stick_x * mult,
                true);
        follower.update();

        Pose p = follower.getPose();
        if (p != null)
        {
            follower.setPose(new Pose(
                    p.getX() - driftcorrx * (1.0 / 30.0),
                    p.getY() - driftcorry * (1.0 / 30.0),
                    p.getHeading() - driftcorrh * (1.0 / 30.0)));
        }

        regressions();
        dipshot();
        runballdetection();
        shootfast();
        activeintake();
        updatehold();

        Pose fused = follower.getPose();
        if (fused != null)
        {
            double dx = targetx - fused.getX(), dy = targety - fused.getY();
            telemetrym.addData("dist", Math.hypot(dx, dy));
        }

        if (dbzGamepad1.x)
        {
            follower.setPose(new Pose(129, 111, Math.toRadians(90)));
            turretoffset = 0;
        }
        if (dbzGamepad1.y)
        {
            follower.setPose(new Pose(9.76378, 8.661, Math.toRadians(180)));
            turretoffset = 0;
        }

        aim();
        runflywheel();
        sendtelem();

        telemetrym.update(telemetry);
        telemetry.update();
    }

    private void updatelights()
    {
        if (blinkin == null) return;
        double pos = ballslocked ? 0.722 : (intakerev ? 0.277 : 0.0);
        pos = Math.round(pos * 1000.0) / 1000.0;
        if (Math.abs(lastlightpos - pos) > 0.001)
        {
            blinkin.setPosition(pos);
            lastlightpos = pos;
        }
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

        switch (ballstate)
        {
            case IDLE:
                if (hit && !shoot)
                {
                    if (!prevdetect) { detecttimer.reset(); prevdetect = true; }
                    if (detecttimer.seconds() >= 0.2)
                    {
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
                }
                else if (!hit)
                {
                    prevdetect = false;
                    ballslocked = false;
                }
                break;

            case REVERSING:
                holdoverride = false;
                holdposition = holdclose;
                if (!shoot && revtimer.seconds() < 3.0)
                {
                    lpush.setPosition(lockpos);
                    rpush.setPosition(lockpos - servooff);
                    intake.setPower(-1);
                }
                if (revtimer.seconds() >= 0.5) holdposition = holdopen;
                if (revtimer.seconds() >= 3.0)
                {
                    intake.setPower(-1);
                    lpush.setPosition(push0);
                    rpush.setPosition(push0 - servooff);
                    ballstate = BallState.LOCKED;
                }
                break;

            case LOCKED:
                lpush.setPosition(lockpos);
                rpush.setPosition(lockpos - servooff);
                if (shoot) intake.setPower(1);
                break;
        }
    }

    private void regressions()
    {
        Pose p = follower.getPose();
        if (p == null) return;
        if (autohood)
        {
            Pose vgoal = virtualgoal(p);
            double dist = Math.hypot(vgoal.getX() - p.getX(), vgoal.getY() - p.getY());
            hoodbase = Math.max(0.0, Math.min(1.0, hooda*dist*dist + hoodb*dist + hoodc));
            targetvelocity = Math.max(-maxvelocity, Math.min(maxvelocity, vela*dist*dist + velb*dist + velc));
        }
        else
        {
            hoodbase = hooddefault;
            targetvelocity = manualvel;
        }
    }

    private void dipshot()
    {
        if (shoot && !dipping && !dipdone) { dipping = true; diptimer.reset(); }
        if (!shoot)
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
        com.pedropathing.math.Vector vel = follower.getVelocity();
        double vx = vel != null ? vel.getXComponent() : 0.0;
        double vy = vel != null ? vel.getYComponent() : 0.0;
        if (Math.hypot(vx, vy) < 1.5) { vx = 0; vy = 0; }
        double dist = Math.hypot(goalx - p.getX(), goaly - p.getY());
        double shottime = timea*dist*dist + timeb*dist + timec;
        return new Pose(goalx - vx*shottime, goaly - vy*shottime, 0);
    }

    private void shootfast()
    {
        boolean trig = dbzGamepad1.right_trigger > 0.1;

        if (!fastmode)
        {
            if (trig && !lastshoot && !shoot)
            {
                boolean had3 = ballslocked;
                boolean holdready = hold.getPosition() >= holdopen - 0.01;
                holdoverride = false;
                holdposition = holdclose;
                intaketimer.reset();
                shoot = true;
                ballslocked = false;
                dipping = false;
                dipdone = false;
                if (had3 && !holdready)
                {
                    holdwait = true;
                    holdtimer.reset();
                }
                else
                {
                    holdwait = false;
                    lpush.setPosition(push1);
                    rpush.setPosition(push1 - servooff);
                    intaketimer.reset();
                }
            }
            if (shoot && holdwait)
            {
                if (holdtimer.milliseconds() >= 200)
                {
                    holdwait = false;
                    lpush.setPosition(push1);
                    rpush.setPosition(push1 - servooff);
                    intaketimer.reset();
                }
            }
            else if (shoot)
            {
                if (intaketimer.milliseconds() > shotret)
                {
                    lpush.setPosition(push0);
                    rpush.setPosition(push0 - servooff);
                    holdposition = holdclose;
                    shoot = false;
                    resetshot();
                }
                else if (intaketimer.milliseconds() > shot2)
                {
                    lpush.setPosition(push3);
                    rpush.setPosition(push3 - servooff);
                }
                else if (intaketimer.milliseconds() > shot1)
                {
                    lpush.setPosition(push2);
                    rpush.setPosition(push2 - servooff);
                }
            }
        }
        else
        {
            if (trig && !lastshoot && !shoot)
            {
                boolean had3 = ballslocked;
                boolean holdready = hold.getPosition() >= holdopen - 0.01;
                ballslocked = false;
                holdoverride = false;
                holdposition = holdopen;
                intaketimer.reset();
                shoot = true;
                dipping = false;
                dipdone = false;
                if (had3 && !holdready)
                {
                    holdwait = true;
                    holdtimer.reset();
                }
                else
                {
                    holdwait = false;
                    lpush.setPosition(push3);
                    rpush.setPosition(push3 - servooff);
                    intaketimer.reset();
                }
            }
            if (shoot && holdwait)
            {
                if (holdtimer.milliseconds() >= 200)
                {
                    holdwait = false;
                    lpush.setPosition(push3);
                    rpush.setPosition(push3 - servooff);
                    intaketimer.reset();
                }
            }
            else if (shoot && intaketimer.milliseconds() > 700)
            {
                lpush.setPosition(push0);
                rpush.setPosition(push0 - servooff);
                holdposition = holdclose;
                shoot = false;
                resetshot();
            }
            if (!shoot) intaketimer.reset();
        }

        lastshoot = trig;
        telemetry.addData("fastmode", fastmode);
        telemetry.addData("holdwait", holdwait);
    }

    private void resetshot()
    {
        ballslocked = false;
        ballstate = BallState.IDLE;
        holdoverride = false;
        holdposition = holdclose;
        holdwait = false;
        lpush.setPosition(push0);
        rpush.setPosition(push0 - servooff);
        intakefwd = true;
        intakerev = false;
        intake.setPower(1);
    }

    private void activeintake()
    {
        boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;
        if (shoot) { intake.setPower(0); lastrb = rb; lastlb = lb; return; }
        if (ballslocked) { intake.setPower(-1); lastrb = rb; lastlb = lb; return; }

        if (rb && !lastrb) { intakefwd = !intakefwd; intakerev = false; }
        if (lb && !lastlb) { intakerev = !intakerev; intakefwd = false; }

        intake.setPower(intakefwd ? 1 : intakerev ? -1 : 0);

        if (!ballslocked && !shoot)
        {
            lpush.setPosition(push0);
            rpush.setPosition(push0 - servooff);
        }
        lastrb = rb;
        lastlb = lb;
    }

    private void updatehold()
    {
        if (!holdoverride)
        {
            if (shoot)
                holdposition = holdopen;
            else if (ballstate == BallState.REVERSING)
                holdposition = revtimer.seconds() >= 0.5 ? holdopen : holdclose;
            else if (ballstate == BallState.LOCKED)
                holdposition = holdopen;
        }
        hold.setPosition(holdposition);
    }

    private void aim()
    {
        boolean aimBtn = gamepad1.dpad_right;
        if (aimBtn && !lastaim) { aiming = !aiming; tpid.reset(); }
        lastaim = aimBtn;

        double tgtangle;
        if (!aiming)
        {
            turretstate = TurretState.NORMAL;
            tgtangle = 0.0;
        }
        else
        {
            double clamped = clampturret();
            switch (turretstate)
            {
                case NORMAL:
                    if (Math.abs(clamped - getturretdeg()) <= thresh)
                        tgtangle = clamped;
                    else
                    {
                        turretstate = TurretState.CENTERING;
                        tgtangle = 0.0;
                    }
                    break;
                case CENTERING:
                    tgtangle = 0.0;
                    if (Math.abs(getturretdeg()) < 5.0) turretstate = TurretState.NORMAL;
                    break;
                default:
                    tgtangle = 0.0;
                    turretstate = TurretState.NORMAL;
            }
        }

        double cur = getturretdeg();
        targetdeg = tgtangle;
        currentdeg = cur;
        double err = wrapangle(tgtangle - cur);
        turretontarget = Math.abs(err) < 2.0;

        if (Math.abs(err) <= tdead) { turret.setPower(0); return; }

        tpid.setPID(tkp, tki, tkd);
        double out = tpid.calculate(cur, tgtangle);
        double ff = Math.abs(err) > tffdead ? Math.copySign(tks, err) : 0.0;
        turret.setPower(Math.max(-tmax, Math.min(tmax, out + ff)));
    }

    private void runflywheel()
    {
        flytarget = targetvelocity;
        flycurrent = fly2.getVelocity();

        if (Math.abs(targetvelocity) <= 1.0)
        {
            fly1.setPower(0);
            fly2.setPower(0);
            velontarget = false;
            return;
        }

        double maxvel = fly2.getMotorType().getMaxRPM() * fly2.getMotorType().getTicksPerRev() / 60.0;
        double batv = Math.max(10.5, vsensor.getVoltage());
        double ff = bangff * (targetvelocity / maxvel) * (12.0 / batv);
        double bb = flycurrent < targetvelocity ? 1.0 : 0.0;
        double power = Math.min(1.0, bb + ff);

        fly1.setPower(power);
        fly2.setPower(power);
        velontarget = Math.abs(targetvelocity - flycurrent) < 40.0;
    }

    private double getturretdeg()
    {
        double angle = (tenc.getVoltage() / tenc.getMaxVoltage()) * 360.0 - tzero;
        return wrapasym(angle, thresh);
    }

    private double getdesiredturretdeg()
    {
        Pose p = follower.getPose();
        if (p == null) return getturretdeg();
        Pose vgoal = virtualgoal(p);
        double fieldangle = Math.atan2(vgoal.getY() - p.getY(), vgoal.getX() - p.getX());
        return wrapasym(Math.toDegrees(fieldangle - p.getHeading()) + turretoffset, thresh);
    }

    private double clampturret()
    {
        double d = wrapasym(getdesiredturretdeg(), thresh);
        if (d > thresh2) return thresh2;
        if (d < -thresh) return -thresh;
        return d;
    }

    private double wrapangle(double a) { return ((a + 180) % 360 + 360) % 360 - 180; }
    private double wrapasym(double a, double n) { return ((a + n) % 360 + 360) % 360 - n; }

    private void sendtelem()
    {
        telemetry.addData("target degrees: ", targetdeg);
        telemetry.addData("current degrees: ", currentdeg);
        telemetry.addData("turret error: ", wrapangle(targetdeg - currentdeg));
        telemetry.addData("turret on target: ", turretontarget);
        telemetry.addData("intake current: ", String.format("%.2f", intake.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("flywheel target velo: ", flytarget);
        telemetry.addData("flywheel current velo: ", flycurrent);
        telemetry.addData("flywheel error: ", flytarget - flycurrent);
        telemetry.addData("on target? ", velontarget);
        telemetry.addData("maxvmode: ", rpmmode ? "rpm" : "1900");
        telemetry.addData("maxvelocity: ", maxvelocity);
        telemetry.addData("hold pos: ", holdposition);
        telemetry.addData("hold override: ", holdoverride);
        telemetry.addData("hold state: ", ballstate);
    }

    @Override public void opLoopHook() {}
    @Override public void opTeardown() {}
}