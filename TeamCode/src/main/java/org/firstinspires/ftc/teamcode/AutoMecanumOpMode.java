package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by chscompsci on 2/17/2017.
 */

public abstract class AutoMecanumOpMode extends MecanumOpMode {
    public ColorSensor color1;

    private boolean pressedBeacon = false;
    private int numTimes=0;

    public void detectColor() throws InterruptedException {
        numTimes++; // Calculate number of times this method has been called
        telemetry.addData("RGB-Left", color1.red() + ", " + color1.green() + ", " + color1.blue());
        telemetry.addData("Color Sense", color() + " NumTimes: "+numTimes);
        Thread.sleep(500); // Wait for 500 millisecond to detect color
        telemetry.addData("Color Sense", color() + " NumTimes: AFTER 500 MILLI"+numTimes);
        // Checks if pressed beacon and right color
        if (color() && !pressedBeacon) {
            driveAngle(0,1);
           // driveAngle(Math.PI/2, 1);
            Thread.sleep(200);
            driveAngle(0,0);
            pressedBeacon=true;
        }
    }

    //Returns True-Red, False-Blue
    public boolean color() {
        return color1.red() > color1.blue();
    }
}
