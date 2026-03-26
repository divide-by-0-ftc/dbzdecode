package org.firstinspires.ftc.teamcode.extensions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class DbzOpMode extends LinearOpMode {


    protected DbzTelemetry dbzTelemetry;
    protected DbzGamepad dbzGamepad1, dbzGamepad2;
    protected DbzHardwareMap robot;

    private static DbzOpMode instance;

    public DbzOpMode() {
        instance = this;
    }

    @Override
    public void runOpMode() {
        dbzTelemetry = new DbzTelemetry(telemetry);
        dbzGamepad1 = new DbzGamepad(gamepad1);
        dbzGamepad2 = new DbzGamepad(gamepad2);
        robot = new DbzHardwareMap(hardwareMap);

        opInit();
        waitForStart();
        opLoopHook();

        while (opModeIsActive()) {
            opLoop();
            dbzTelemetry.update();
            dbzGamepad1.update();
            dbzGamepad2.update();
        }

        opTeardown();
    }

    static HardwareMap getInstanceHardwareMap() {
        if (instance != null) return instance.hardwareMap;
        return null;
    }

    protected abstract void opInit();
    protected abstract void opLoopHook();
    protected abstract void opLoop();
    protected abstract void opTeardown();
}
