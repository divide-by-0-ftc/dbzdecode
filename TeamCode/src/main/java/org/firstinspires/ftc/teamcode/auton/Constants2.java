package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants2 {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.33771)
            .forwardZeroPowerAcceleration(-30.284583120240786)
            .lateralZeroPowerAcceleration(-80.42751649688425)


//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.013, 0.001))
//            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.01, 0.01))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.16674500515578042, 0.0010742245751442295))
            .centripetalScaling(0);
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.00001,0.5,0.03));





    public static PathConstraints defaultConstraints = new PathConstraints(0.96, 0.1, 0.1, 0.007, 100, 1, 10, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(defaultConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)


                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(77.7877379890502)
            .yVelocity(53.97284368079479);



    public static PinpointConstants localizerConstants = new PinpointConstants()
            //right for tele
//            .forwardPodY(-4.311)
//            .strafePodX(0)
            //right for auto
            .forwardPodY(-7.9213)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
