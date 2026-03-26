package org.firstinspires.ftc.teamcode.extensions;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DbzRev2mDistanceSensor implements DbzDevice {
    private Rev2mDistanceSensor distanceSensor;
    public DbzRev2mDistanceSensor(Rev2mDistanceSensor distanceSensor){
        this.distanceSensor = distanceSensor;
    }
    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.METER);
    }

}
