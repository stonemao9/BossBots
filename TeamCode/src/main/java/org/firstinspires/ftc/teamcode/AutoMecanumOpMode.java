package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;

import static java.lang.Double.NaN;

/**
 *Created by chscompsci on 2/17/2017.
 */

public abstract class AutoMecanumOpMode extends MecanumOpMode {
    public ColorSensor color1;

    /* Declare OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor belt;
    public DcMotor eightyTwenty;
    public DcMotor sweeper;
    public Servo idleGear;


    //Phone sensors
    public Sensor magnetometer;
    public Sensor accelerometer;

    //True= Red, False=Blue
    public boolean teamColor;
    public UltrasonicSensor ultra;

    public DcMotor shooter;
    public ColorSensor colorLeft;
    public ColorSensor colorRight;
    public Servo ballKeeper;
    public Servo flicker;
    public double initAfterRT;
    public CompassSensor compass;
    public VoltageSensor vltageSensor;
    public DcMotor autoShooter;


    //PID variables
    public double setx, curx, lastx, tottotx, totx, velx, kp, kd, outx, dx, startingEncoderMotor2, startingEncoderMotor4;

    public long interval; //sensor sample period (1/sample frequency)

    private boolean pressedBeacon = false;
    private int numTimes = 0;

    public void detectColor() throws InterruptedException {
        numTimes++; // Calculate number of times this method has been called
        telemetry.addData("RGB-Left", color1.red() + ", " + color1.green() + ", " + color1.blue());
        telemetry.addData("Color Sense", color() + " NumTimes: "+numTimes);
        Thread.sleep(500); // Wait for 500 millisecond to detect color
        telemetry.addData("Color Sense", color() + " NumTimes: AFTER 500 MILLI"+numTimes);
        // Checks if pressed beacon and right color
        if (color() == teamColor && !pressedBeacon) {
            driveAngle(0,1);
            // driveAngle(Math.PI/2, 1);
            Thread.sleep(200);
            driveAngle(0,0);
            pressedBeacon = true;
        }
    }

    //Returns True-Red, False-Blue
    public boolean color() {
        return color1.red() > color1.blue();

    }

    //takes in setpoint, takes robot forward, returns motor power
    public double goToPosition(double setpointx) {
        setx = setpointx; //x position that is not changing
        final double CIRCUMFERENCE = 0.618422514*2; //DO NOT CHANGE
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

            if (outx == NaN) {
                outx = 0;
            }
            telemetry.addData("outx", outx);
            driveAngle(Math.PI / 2, outx);
        }
        telemetry.addData("outX", Math.round(outx * 100) / (double) 100);
        return Math.round(outx * 100) / (double) 100;
    }

    public double lastAng;

    public double compassReadingInitial;
    public double ppcurrentAngle;
    public int n = 0; //counts how many times compass.getdirection() has reset
    public double currentAngularPosition;
    public double setpoint;
    public boolean setOnce = true;
    //turn the robot by angle in DEGREES
    public double turnByAngle(double setAngle) throws InterruptedException {
        if(setOnce){
            currentAngle();
            setpoint = currentAngularPosition + setAngle;
            setOnce = false;
        }

        double curang = currentAngularPosition;
        double errAngle = setpoint - curang;
//        double changInAngle = angleZ; //get it from the Modern Robotics Gyro (given up on AdaFruit)
//        curang += changInAngle;

        double kpAngle = 0.01;
        double kdAngle = 0.0;

        double velAngle = (kdAngle * (curang - lastAng)) / interval;
        double outAngle = (kpAngle*errAngle) - velAngle;

        if (outAngle >= 1){
            outAngle = 1;
        }

        if (outAngle <= -1){
            outAngle = -1;
        }

        if (outAngle == NaN){
            outAngle = 0;
        }

        motor1.setPower(outAngle);
        motor2.setPower(-1 * outAngle);
        motor3.setPower(-1 * outAngle);
        motor4.setPower(outAngle);

        lastAng = curang;

        telemetry.addData("Current Angle", curang);
        telemetry.addData("Error in Angle", errAngle);
        telemetry.addData("Motor Output", "kpangle ("+kpAngle+") - "+outAngle);
        telemetry.addData("Current Direction", currentAngularPosition);

        return Math.round(outx * 10) / (double) 10;
    }

    public void currentAngle() throws InterruptedException {
        double compassReadingCurrent = compass.getDirection();
        double changeInAngle = compassReadingCurrent - compassReadingInitial;

        ppcurrentAngle += changeInAngle;

        if (ppcurrentAngle > 320){
            ppcurrentAngle = 360 - ppcurrentAngle;
            n++;
        }

        compassReadingInitial = compassReadingCurrent;

        currentAngularPosition = (n * 360) + ppcurrentAngle;
        Thread.sleep(interval);
    }

    public void shoot(){

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
    public boolean done=false;
    public void driveRight() {
            motor1.setPower(0.5);
            motor2.setPower(-0.15);
            motor3.setPower(0.5);
            motor4.setPower(-0.15);
    }

    public void hardAutoShooter(){
        autoShooter.setPower(0.5);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}