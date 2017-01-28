/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Team 524 Autonomous", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class NewFinalAutonomous524 extends MecanumOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor belt;
    private DcMotor eightyTwenty;
    private DcMotor sweeper;
    private Servo idleGear;


    //Phone sensors
    private double curX;
    private double curY;

    private String teamColor;
    private DcMotor shooter;
    private AccelerationSensor accelNXT;
    private GyroSensor gyroNXT;
    private ColorSensor color;
    private Servo ballKeeper;
    private Servo flicker;
    private double initAfterRT;

    //PID variables
    private double setx, curx, lastx, tottotx, totx, velx, kp, kd, outx, dx, startingEncoderMotor2, startingEncoderMotor4;
    private double setDist;

    private long interval; //sensor sample period (1/sample frequency)

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
    private double[] coord = new double[3];


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //set sample period for sensors
        interval = SensorManager.SENSOR_DELAY_GAME;
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);
        idleGear = hardwareMap.servo.get("idleGear");
        teamColor = "b";

        kp = 0.0762;
        kd = 0.0;

        //NXT
        accelNXT = hardwareMap.accelerationSensor.get("acc");
        gyroSense = hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        color.enableLed(false);
        shooter = hardwareMap.dcMotor.get("shooter");

        currentHeading = gyroSense.getHeading();
        ballKeeper = hardwareMap.servo.get("ballKeeper");
        flicker = hardwareMap.servo.get("flicker");
        flicker.setPosition(0.55);
        ballKeeper.setPosition(0.0);
        telemetry.addData("Status", "Initialized");
        initAfterRT = runtime.milliseconds();
        startingEncoderMotor2 = motor2.getCurrentPosition();
        idleGear.setPosition(0.72);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("startingEncoderMotor2", startingEncoderMotor2);
        telemetry.addData("RGB", color.red() + ", " + color.green() + ", " + color.blue());
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        totx = tottotx;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    private boolean task0 = false;
    private boolean task1 = false;
    private boolean task2 = false;
    private boolean task3 = false;
    private boolean task4 = false;
    private boolean task5 = false;
    private boolean task6 = false;
    private boolean task7 = false;
    private boolean task8 = false;
    private boolean task9 = false;
    private boolean task10 = false;
    private boolean task11 = false;
    private boolean task12 = false;
    private boolean task13 = false;
    private boolean task14 = false;
    private boolean task15 = false;
    private boolean task16 = false;
    private double tempTime = 0;

    @Override
    public void loop() {
        telemetry.addData("RGB", color.red() + ", " + color.green() + ", " + color.blue());

//        if (!task0) {
//            if (runtime.milliseconds() < 600) {
//                driveAngle(Math.PI / 2, 0.7);
//            } else {
//                driveAngle(Math.PI / 2, 0);
//                tempTime = runtime.milliseconds();
//                task0 = true;
//            }
//            telemetry.addData("Task", "Forward");
//        } else if (!task1) {
//            if (runtime.milliseconds() < tempTime + 600) {
//                motor1.setPower(-0.5);
//                motor2.setPower(0.5);
//                motor3.setPower(0.5);
//                motor4.setPower(-0.5);
//            } else {
//                motor1.setPower(0);
//                motor2.setPower(0);
//                motor3.setPower(0);
//                motor4.setPower(0);
//                tempTime = runtime.milliseconds();
//                task1 = true;
//            }
//            telemetry.addData("Task", "Turning");
//        } else if (!task2) {
//            totx = tottotx;
//            task2 = true;
//
         if (!task3) {
            if (goToPosition(1.52) < 0.13) {
                task3 = true;
                driveAngle(0, 0);
                tempTime = runtime.milliseconds();
            }
            telemetry.addData("totx", totx);
            telemetry.addData("tottotx", tottotx);
            telemetry.addData("Task", "Moving towards beacon");
        } //else if (!task4) {
//            if (runtime.milliseconds() < tempTime + 600) {
//                motor1.setPower(0.5);
//                motor2.setPower(-0.5);
//                motor3.setPower(-0.5);
//                motor4.setPower(0.5);
//            } else {
//                motor1.setPower(0);
//                motor2.setPower(0);
//                motor3.setPower(0);
//                motor4.setPower(0);
//                tempTime = runtime.milliseconds();
//                task4 = true;
//            }
//            telemetry.addData("Task", "Turning");
//        } else if (!task5) {
//            if (runtime.milliseconds() < tempTime + 1100) {
//                driveAngle(1 * Math.PI / 180, 0.6);
//            } else {
//                driveAngle(0, 0);
//                tempTime = runtime.milliseconds();
//                task5 = true;
//            }
//            telemetry.addData("Task", "Moving at zero degrees");
//        } else if (!task8) {
//            telemetry.addData("color!!!", sameColor(teamColor, color));
//            if (sameColor(teamColor, color)) {
//                if (runtime.milliseconds() < tempTime + 600) {
//                    driveAngle(0, 1);
//                } else {
//                    driveAngle(0, 0);
//                    task8 = true;
//                    task9 = true;
//                    task10 = true;
//                }
//            } else {
//                telemetry.addData("color123123", false);
//                task8 = true;
//            }
//        } else if (!task9) {
//            totx = tottotx;
//            task9 = true;
//        } else if (!task10) {
//            telemetry.addData("totx", totx);
//            telemetry.addData("Task", "Moving away from wrong color");
//            telemetry.addData("tottotx", tottotx);
//            if (goToPosition(-0.3) < 0.13) {
//                task10 = true;
//                driveAngle(0, 0);
//                tempTime = runtime.milliseconds();
//            }
//        }
    }

    private void press(double tempTime) {
        if (runtime.milliseconds() < tempTime + 600) {
            driveAngle(0, 1);
        } else {
            driveAngle(0, 0);
        }
    }

    private boolean sameColor(String teamColor, ColorSensor color) {
        if (teamColor.equals("r")) {
            if (color.red() > color.blue()) {
                return true;
            } else {
                return false;
            }
        } else {
            if (color.red() < color.blue()) {
                return true;
            } else {
                return false;
            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public double goToPosition(double setpointx) {
        setx = setpointx; //x position that is not changing
        final double CIRCUMFERENCE = 0.700459; //DO NOT CHANGE
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

    public double dotProduct(double[] vector1, double[] vector2) {
        return (vector1[0] * vector2[0]) + (vector1[1] * vector2[1]) + (vector1[2] * vector2[2]);
    }

    public void takeMeTo(double x, double y){
        double distance = Math.sqrt(Math.pow((curX - x),2) + Math.pow((curY - y),2));

        goToPosition(distance);
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
}