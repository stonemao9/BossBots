package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by chscompsci on 2/17/2017.
 */

public abstract class AutoMecanumOpMode extends MecanumOpMode {
    public ColorSensor color1;
    public double setx, curx, lastx, tottotx, totx, velx, kp, kd, outx, dx, startingEncoderMotor2;
    public long interval; //sensor sample period (1/sample frequency)
    public ElapsedTime runtime;
    public boolean teamColor; // True = red False

    //takes in setpoint, takes robot forward, returns motor power
    public double goToPosition(double setpointx) {
        setx = setpointx; //x position that is not changing
        final double CIRCUMFERENCE = 0.618422514; //DO NOT CHANGE
        tottotx = ((((motor2.getCurrentPosition() - startingEncoderMotor2))) / 1426) * CIRCUMFERENCE;

        //should it be <= someNumber instead of ==someNumber? (will the code stop when getRuntime()%interval != 0?)
        if (runtime.milliseconds() % interval <= 19) {
            curx = tottotx - totx;

            velx = (kd * (curx - lastx)) / interval;

            outx = (kp * (setx - curx)) - velx;

            if (outx >= 1) {
                outx = 1;
            }

            if (outx <= -1) {
                outx = -1;
            }
            telemetry.addData("outx", outx);
            driveAngle(Math.PI / 2, outx);
        }
        telemetry.addData("outX", Math.round(outx * 10) / (double) 10);
        return Math.round(outx * 10) / (double) 10;
    }

    double lastAng;

    //turn the robot by angle in RADIANS
    public void turnByAngle(double setAngle) {
        double curang = 0;
        double errAngle = setAngle - curang;
        double changInAngle = angleZ; //get it from the Modern Robotics Gyro (given up on AdaFruit)
        curang += changInAngle;

        double kpAngle = 0.01;
        double kdAngle = 0.0;

        double velAngle = (kdAngle * (curang - lastAng)) / interval;
        double outAngle = (kpAngle * errAngle) - velAngle;

        if (outAngle >= 1) {
            outAngle = 1;
        }

        if (outAngle <= -1) {
            outAngle = -1;
        }

        lastAng = curang;

        telemetry.addData("Current Angle", curang);
        telemetry.addData("Error in Angle", errAngle);
        telemetry.addData("Motor Output", outAngle);
    }

    public double dotProduct(double[] vector1, double[] vector2) {
        return (vector1[0] * vector2[0]) + (vector1[1] * vector2[1]) + (vector1[2] * vector2[2]);
    }

    public double[] crossProduct(double[] vector1, double[] vector2) {
        double[] cp = new double[3];
        cp[0] = (vector1[1] * vector2[2]) - (vector1[2] * vector2[1]);
        cp[1] = (vector1[2] * vector2[0]) - (vector1[0] * vector2[2]);
        cp[2] = (vector1[0] * vector2[1]) - (vector1[1] * vector2[0]);

        return cp;
    }

    public double[] unitVector(double[] vector1) {
        double[] uv = new double[3];

        for (int i = 0; i <= 2; i++) {
            uv[i] = vector1[i] / dotProduct(vector1, vector1);
        }
        return uv;
    }
/*
*
* COLOR SENSORS
*
*/

    private boolean pressedBeacon = false;
    private boolean notTask = false;
    private int numTimes = 0;
    private int cw;
    public void detectColor() throws InterruptedException {
        numTimes++; // Calculate number of times this method has been called
        telemetry.addData("RGB-Left", color1.red() + ", " + color1.green() + ", " + color1.blue());
        telemetry.addData("Color Sense", color() + " NumTimes: " + numTimes);
        Thread.sleep(500); // Wait for 500 millisecond to detect color
        telemetry.addData("Color Sense", color() + " NumTimes: AFTER 500 MILLI" + numTimes);
        // Checks if pressed beacon and right color
        if (color() == teamColor && !pressedBeacon) {
            cw=1;
        } else if (color() == teamColor && !pressedBeacon) {
            cw=-1;
        }
        if(!pressedBeacon){
            driveAngle(Math.PI/2,1);
            Thread.sleep(200);
            Thread.sleep(200);
            hardcodeTurning(cw);
            Thread.sleep(300);
            driveAngle(0, 0);
            pressedBeacon = true;
            hardcodeTurning(-cw);
            Thread.sleep(200);
            driveAngle(0, 0);
        }

    }

    //Returns True-Red, False-Blue
    public boolean color() {
        return color1.red() > color1.blue();
    }
}
