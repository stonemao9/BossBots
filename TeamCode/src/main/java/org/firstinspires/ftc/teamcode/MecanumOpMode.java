package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/**
 * Created by Stone Mao & Cooper LaRhette in the 2016-2017 school year
 * for the Campolindo High School robotics team The Boss Bots
 */

 /*
    *   Motor position
    *
    * motor4     motor3
    *    []-------[]
    *      |     |
    *      |     |
    *      |     |
    *    []-------[]
    *  motor1    motor2
    */

public abstract class MecanumOpMode extends OpMode implements SensorEventListener {

    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;

    public String teamColor;
    
    public SensorManager sensorService;
    public AccelerationSensor accSense;
    public GyroSensor gyroSense;

    public double accX;
    public double accY;
    public double accZ;

    public double gyroX;
    public double gyroY;
    public double gyroZ;


    private boolean slowMode = false;
    //    final double CHANGE = 0.09; //rate of change, used in accl
//    double limitedLength = 0; // Power truncation/factor, used in accl
    double previousAngle = 0;
    double angle, length;

    public void turning() {
        motor1.setPower(-gamepad1.right_stick_x);
        motor2.setPower(gamepad1.right_stick_x);
        motor3.setPower(gamepad1.right_stick_x);
        motor4.setPower(-gamepad1.right_stick_x);
    }

    /**
     * Drive the holonomic drivetrain with one joystick
     *
     * @param gamepad Which gamepad to use: 1 or 2. Default 1
     * @param side    Which joystick to use: "left" or "right." Default "right"
     */
    public void driveOneJoystick(Gamepad gamepad, String side) {
        if (side.equalsIgnoreCase("left")) {
            angle = this.getJoystickAngle(gamepad.left_stick_x, gamepad.left_stick_y);
            length = this.getDistance(gamepad.left_stick_x, gamepad.left_stick_y);
        } else {
            angle = this.getJoystickAngle(gamepad.right_stick_x, gamepad.right_stick_y);
            length = this.getDistance(gamepad.right_stick_x, gamepad.right_stick_y);
        }

        /**
         * Acceleration & Deceleration code --> inactive
         * if we re-implement remeber to change power to be based of of limitedLength
         * instead of length
         */
//        if (limitedLength+CHANGE <= length) { // Player pulls joystick to full extent
//            limitedLength += CHANGE; // Factor increases to accelerate
//            previousAngle = angle; // Stores the angle which the robot is accelerating at
//        } else if (length<=0.2) { // Joystick released to 0,0
//            if(limitedLength-CHANGE > length){ // Checks to see if it still can decelerate
//                limitedLength -= CHANGE;
//            } else {
//                limitedLength=0;
//            }
//            angle = previousAngle; // So it doesn't go right (0 deg) when we joystick is back at 0,0
//        }

        //Calculates the motor power based off of trignometric functions
        double sin2and4 = Math.abs(getLength()) * Math.round(Math.sin(angle - Math.PI / 4) * 10.0) / 10.0;
        double cos1and3 = Math.abs(getLength()) * Math.round(Math.cos(angle - Math.PI / 4) * 10.0) / 10.0;

        //Driving
        if (Math.abs(gamepad.right_stick_x) != 0) {
            turning();
        } else {
            motor1.setPower(cos1and3);
            motor2.setPower(sin2and4);
            motor3.setPower(cos1and3);
            motor4.setPower(sin2and4);
        }

        telemetry.addData("sin", sin2and4);
        telemetry.addData("cos", cos1and3);

    }

    public void driveAngle(double angle, double scale) {
        double sin2and4 = scale * Math.round(Math.sin(angle - Math.PI / 4) * 10.0) / 10.0;
        double cos1and3 = scale * Math.round(Math.cos(angle - Math.PI / 4) * 10.0) / 10.0;
        motor1.setPower(cos1and3);
        motor2.setPower(sin2and4);
        motor3.setPower(cos1and3);
        motor4.setPower(sin2and4);
    }


    /**
     * Calculate the angle which the joystick is currently at
     *
     * @param x the x position of the joystick
     * @param y the y position of the joystick
     */
    public double getJoystickAngle(double x, double y) {
        //First Figure out the Quadrant then find the angle
        if (-y >= 0) {
            return Math.atan2(-y, x);
        } else if (-y < 0) {
            return 2 * Math.PI + Math.atan2(-y, x);
        }
        return 0;
    }

    /**
     * Calculate the distance from the center to where the joystic is currently at
     *
     * @param x the x position of the joystick
     * @param y the y position of the joystick
     */
    public double getDistance(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    /**
     * method may modify length to be smaller if slowMode is toggled on by left bumper
     *
     * @return length which is the power multiple for motors
     */
    public double getLength() {
        if (gamepad1.left_bumper)
            slowMode = true;
        else if (gamepad1.right_bumper)
            slowMode = false;
        if (!slowMode)
            return length;
        else
            return length = length / 2;
    }


    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Ignoring this for now
    }
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
//        switch (sensorEvent.sensor.getType()) {
//            case Sensor.TYPE_ACCELEROMETER:
//                accX = sensorEvent.values[0];
//                accY = sensorEvent.values[1];
//                accZ = sensorEvent.values[2];
//                break;
//        }
        Acceleration a = accSense.getAcceleration();
        accX = a.xAccel;
        accY = a.yAccel;
        accZ = a.zAccel;

        gyroX = gyroSense.rawX();
        gyroY = gyroSense.rawY();
        gyroZ = gyroSense.rawZ();
    }
}
