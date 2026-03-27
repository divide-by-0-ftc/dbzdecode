package org.firstinspires.ftc.teamcode;

public class notes {

    /// PID CONTROLLER LOGIC
//    public static double vkP = 5, vkF = 1.2, vkD = 0.0, vkDMax = 0.25;
//
//    private double lastVelErr = 0.0, lastVelTime = 0.0;
//    private ElapsedTime veltimer = new ElapsedTime();
    /// CLASS TO ADD
//    private void runflywheel()
//    {
//        flytarget = targetvelocity;
//        flycurrent = fly2.getVelocity();
//
//        if (Math.abs(targetvelocity) <= 1.0)
//        {
//            fly1.setPower(0);
//            fly2.setPower(0);
//            velontarget = false;
//            lastVelErr = 0.0;
//            lastVelTime = veltimer.seconds();
//            return;
//        }
//
//        double maxvel = fly2.getMotorType().getMaxRPM() * fly2.getMotorType().getTicksPerRev() / 60.0;
//        double now = veltimer.seconds();
//        double dt = Math.max(1e-3, Math.min(0.1, now - lastVelTime));
//
//        double normerr = (targetvelocity - flycurrent) / maxvel;
//        double pterm = vkP * normerr;
//        double derr = (normerr - lastVelErr) / dt;
//        double dterm = Math.max(-vkDMax, Math.min(vkDMax, vkD * derr));
//
//        double batv = Math.max(10.5, vsensor.getVoltage());
//        double ff = vkF * (targetvelocity / maxvel) * (12.0 / batv);
//
//        double power = Math.max(-1.0, Math.min(1.0, pterm + dterm + ff));
//        fly1.setPower(power);
//        fly2.setPower(power);
//        velontarget = Math.abs(targetvelocity - flycurrent) < 40.0;
//
//        lastVelErr = normerr;
//        lastVelTime = now;

/// KALMAN CLASS
//    private static class KalmanAxis
//    {
//        double x, p, q, r;
//        KalmanAxis(double initx, double q, double r) { this.x = initx; this.p = 1.0; this.q = q; this.r = r; }
//        void predict(double u) { x += u; p += q; }
//        void update(double z) { double k = p / (p + r); x += k * (z - x); p *= (1.0 - k); }
//    }
//
//    public static double kq = 0.01, kr = 2.0, kqh = 0.005, krh = 0.5;
//
//    private KalmanAxis kx, ky, kh;
//    private Pose lastpedropose = null;
    /// KALMAN INIT
//    Pose start = PoseCache.lastPose;
//    kx = new KalmanAxis(start.getX(), kq, kr);
//    ky = new KalmanAxis(start.getY(), kq, kr);
//    kh = new KalmanAxis(start.getHeading(), kqh, krh);
//    lastpedropose = start;
//    /// kalman loop
//    private void updatekalman()
//    {
//        Pose pedro = follower.getPose();
//        if (pedro == null) return;
//
//        if (lastpedropose != null)
//        {
//            kx.predict(pedro.getX() - lastpedropose.getX());
//            ky.predict(pedro.getY() - lastpedropose.getY());
//            kh.predict(wrapangle(pedro.getHeading() - lastpedropose.getHeading()));
//        }
//        lastpedropose = pedro;
//        follower.setPose(new Pose(kx.x, ky.x, kh.x));
//    }
//
//    private Pose getfusedpose()
//    {
//        return new Pose(kx.x, ky.x, kh.x);
//    }
//
//
//    }
//    /// replace drift correction with updatekalman();
//    telemetry.addData("kalman/x", kx.x);
//telemetry.addData("kalman/y", ky.x);
//telemetry.addData("kalman/hdeg", Math.toDegrees(kh.x));
}
