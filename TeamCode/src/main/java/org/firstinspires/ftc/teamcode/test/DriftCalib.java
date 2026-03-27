package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auton.Constants;
import org.firstinspires.ftc.teamcode.extensions.DbzOpMode;

@Config
@TeleOp(name = "DriftCalib")
public class DriftCalib extends DbzOpMode
{
    public static double testduration = 30.0;

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();

    private Pose startpose;
    private double driftx = 0, drifty = 0, drifth = 0;
    private double ratex = 0, ratey = 0, rateh = 0;
    private boolean done = false;

    @IgnoreConfigurable
    public static TelemetryManager telemetrym;

    @Override
    public void opInit()
    {
        telemetry = new com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetrym = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
        follower.update();

        startpose = follower.getPose();
        timer.reset();
    }

    @Override
    public void opLoop()
    {
        follower.update();

        Pose current = follower.getPose();
        double elapsed = timer.seconds();

        if (!done)
        {
            driftx = current.getX() - startpose.getX();
            drifty = current.getY() - startpose.getY();
            drifth = Math.toDegrees(current.getHeading() - startpose.getHeading());

            if (elapsed >= testduration)
            {
                ratex = driftx / elapsed;
                ratey = drifty / elapsed;
                rateh = drifth / elapsed;
                done = true;
            }
        }

        telemetry.addData("elapsed", String.format("%.1f / %.1fs", elapsed, testduration));
        telemetry.addData("drift/x (in)", String.format("%.4f", driftx));
        telemetry.addData("drift/y (in)", String.format("%.4f", drifty));
        telemetry.addData("drift/hdeg",   String.format("%.4f", drifth));

        if (done)
        {
            telemetry.addData("--- RESULTS ---", "");
            telemetry.addData("rate/x (in/s)", String.format("%.5f", ratex));
            telemetry.addData("rate/y (in/s)", String.format("%.5f", ratey));
            telemetry.addData("rate/hdeg/s",   String.format("%.5f", rateh));
            telemetry.addData("correction/x", String.format("%.5f", -ratex));
            telemetry.addData("correction/y", String.format("%.5f", -ratey));
            telemetry.addData("correction/h", String.format("%.5f", Math.toRadians(-rateh)));
            telemetry.addData("", "paste these into V2RED driftcorrx/y/h");
        }

        telemetrym.update(telemetry);
        telemetry.update();
    }

    @Override public void opLoopHook() {}
    @Override public void opTeardown() {}
}