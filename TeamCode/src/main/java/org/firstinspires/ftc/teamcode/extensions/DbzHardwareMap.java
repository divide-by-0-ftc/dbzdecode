package org.firstinspires.ftc.teamcode.extensions;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class DbzHardwareMap {


    public DcMotorEx frontRight, backRight, backLeft, frontLeft;
    //    public DcMotorEx hoodMotor;
    public DcMotorEx intakeMotor;
//    public DcMotorEx outtake1Motor;
//
//    public DcMotorEx outtake2Motor;
//    public DcMotorEx transfer1Motor;
//    public DcMotorEx transfer2Motor;
//    public DcMotorEx flywheelMotor;
//    public ColorSensor colorSensor;
//    public DistanceSensor distanceSensor;
//    public Limelight3A limelight;
//    private Follower follower;


    public Servo holdServo;
    public Servo shoot1Servo;
    public Servo shoot2Servo;
    public Servo hoodServo;
    public DcMotorEx outtake1Motor;
    public DcMotorEx outtake2Motor;
    public DcMotorEx turret;
    public Servo rightpushServo;
    public Servo leftpushServo;
    public AnalogInput turretEncoder;




    public enum Motor {
        frontright("frontRight"),
        backright("backRight"),
        backleft("backLeft"),
        frontleft("frontLeft"),
        //        hood("hoodMotor"),
        intake("intakeMotor"),
        turretEncoder("turretEncoder"),
        outtake1Motor("outtake1Motor"),
        outtake2Motor("outtake2Motor"),
        turret("turret"),
        rightpushServo("rightpushServo"),
        leftpushServo("leftpushServo"),
        hoodServo("hoodServo"),


        holdServo("holdServo");


//        flywheel("flywheelMotor");


        private final String name;
        Motor(String name) { this.name = name; }
        public String getName() { return name; }
    }


    public DbzHardwareMap(HardwareMap hwMap) {
        frontRight = hwMap.get(DcMotorEx.class, Motor.frontright.getName());
        backRight = hwMap.get(DcMotorEx.class, Motor.backright.getName());
        backLeft = hwMap.get(DcMotorEx.class, Motor.backleft.getName());
        frontLeft = hwMap.get(DcMotorEx.class, Motor.frontleft.getName());
        turretEncoder = hwMap.get(AnalogInput.class, "turretEncoder");


        turret = hwMap.get(DcMotorEx.class, Motor.turret.getName());
//
//        hoodMotor = hwMap.get(DcMotorEx.class, Motor.hood.getName());
        intakeMotor = hwMap.get(DcMotorEx.class, Motor.intake.getName());
        rightpushServo = hwMap.get(Servo.class, "rightpushServo");
        leftpushServo = hwMap.get(Servo.class, "leftpushServo");
        hoodServo = hwMap.get(Servo.class, "hoodServo");
        holdServo = hwMap.get(Servo.class, "holdServo");
        outtake1Motor = hwMap.get(DcMotorEx.class, Motor.outtake1Motor.getName());
        outtake2Motor = hwMap.get(DcMotorEx.class, Motor.outtake2Motor.getName());
//        flywheelMotor = hwMap.get(DcMotorEx.class, Motor.flywheel.getName());
//        holdServo = hwMap.get(Servo.class, "holdServo");
//        shoot1Servo = hwMap.get(Servo.class, "shoot1Servo");
//        shoot2Servo = hwMap.get(Servo.class, "shoot2Servo");




        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        limelight = hwMap.get(Limelight3A.class, "limelight");
    }
}


