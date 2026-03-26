package org.firstinspires.ftc.teamcode.extensions;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * DBZ-extended color sensor.
 *
 * @author Jonathan Okasinski
 * @author Brandon Vu
 * @author Nicholas Rizzio
 */

public class DbzColorRangeSensor implements DbzDevice, DistanceSensor, OpticalDistanceSensor,
        LightSensor, ColorSensor
{
    private LynxI2cColorRangeSensor sensor;

    public DbzColorRangeSensor(LynxI2cColorRangeSensor sensor) {
        this.sensor = sensor;
    }

    /**
     * Calculates whether red, green, or blue is the dominant color seen by the color sensor.
     * @return the dominant color
     */
    public Color getDominantColor() {
        int red = red();
        int blue = blue();
        int green = green();

        if (red > blue) {
            if (green > red)
                return Color.GREEN;
            else
                return Color.RED;
        } else {
            if (green > blue)
                return Color.GREEN;
            else
                return Color.BLUE;
        }
    }

    public enum Color {
        RED, BLUE, GREEN, YELLOW, WHITE, UNKNOWN
    }

    /**
     * Calculates whether the color input is more white or yellow.
     * @return white or yellow or unknown
     */
    public Color getYorW() {
        int red = Math.abs(red());
        int blue = Math.abs(blue());

        if (red >= blue * 1.8) {
            return Color.YELLOW;
        } else if ((red / blue) <= 1.2 && (red / blue) > 0.8) {
            return Color.WHITE;
        } else {
            return Color.UNKNOWN;
        }
    }

    @Override
    public int red() {
        return sensor.red();
    }

    @Override
    public int green() {
        return sensor.green();
    }

    @Override
    public int blue() {
        return sensor.blue();
    }

    @Override
    public int alpha() {
        return sensor.alpha();
    }

    @Override
    public int argb() {
        return sensor.argb();
    }

    @Override
    public void enableLed(boolean enable) {
        sensor.enableLed(enable);
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        sensor.setI2cAddress(newAddress);
    }

    @Override
    public I2cAddr getI2cAddress() {
        return sensor.getI2cAddress();
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return sensor.getDistance(unit);
    }

    @Override
    public double getLightDetected() {
        return sensor.getLightDetected();
    }

    @Override
    public double getRawLightDetected() {
        return sensor.getRawLightDetected();
    }

    @Override
    public double getRawLightDetectedMax() {
        return sensor.getRawLightDetectedMax();
    }

    @Override
    public String status() {
        return sensor.status();
    }

    @Override
    public Manufacturer getManufacturer() {
        return sensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return sensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return sensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
