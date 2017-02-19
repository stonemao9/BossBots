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
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Boolean.TRUE;
import static java.lang.Double.NaN;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Team 524 Autonomous", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class FinalAutoRight extends AutoMecanumOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor belt;
    private DcMotor eightyTwenty;
    private DcMotor sweeper;
    private Servo idleGear;
    private Servo etKeeper;


    //Phone sensors
    private Sensor magnetometer;
    private Sensor accelerometer;

    private DcMotor shooter;
    private Servo ballKeeper;
    private Servo flicker;
    private double initAfterRT;

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
        teamColor = false;
        compass = hardwareMap.compassSensor.get("compass");
        ultra = hardwareMap.ultrasonicSensor.get("ultra");
        vltageSensor = hardwareMap.voltageSensor.get("vltageS2ensor");
        autoShooter = hardwareMap.dcMotor.get("shooter");

        etKeeper = hardwareMap.servo.get("etKeeper");
        etKeeper.setPosition(0);

        double batteryVoltage = vltageSensor.getVoltage();
        kp = 0.30 - (batteryVoltage * 0.006);
        kd = 0.02;

        //NXT
        color1 = hardwareMap.colorSensor.get("color");
        color1.enableLed(false);
        shooter = hardwareMap.dcMotor.get("shooter");

        ballKeeper = hardwareMap.servo.get("ballKeeper");
        flicker = hardwareMap.servo.get("flicker");
        flicker.setPosition(0.55);
        ballKeeper.setPosition(0.0);
        telemetry.addData("Status", "Initialized");
        initAfterRT = runtime.milliseconds();
        startingEncoderMotor2 = motor2.getCurrentPosition();

        ballKeeper = hardwareMap.servo.get("ballKeeper");
        flicker = hardwareMap.servo.get("flicker");

        idleGear = hardwareMap.servo.get("idleGear");
        compass = hardwareMap.compassSensor.get("compass");
        ballKeeper.setPosition(0);
        flicker.setPosition(0.55);

        idleGear.setPosition(0.5);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("startingEncoderMotor2", startingEncoderMotor2);
        telemetry.addData("Ultra-",ultra.getUltrasonicLevel());
        telemetry.addData("RGB-Left", color1.red() + ", " + color1.green() + ", " + color1.blue());
        telemetry.addData("Compass",compass.getDirection());
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
    private double ninetydeg;
    private double test2;
    private double tempTime = 0;

    @Override
    public void loop() {
        telemetry.addData("Encoder Flywheel",shooter.getCurrentPosition());
        if (!task0) {
            totx = tottotx;
            task0 = true;
            ninetydeg = compass.getDirection();
        } else if(!task1){
            if(goToPosition(4.2)<0.1){
                task1 = true;
                driveAngle(0,0);
                tempTime = runtime.milliseconds();
            }
            telemetry.addData("Task","Going forward 4.4 ft");
        } else if(!task2){
            if(compass.getDirection()>ninetydeg-11){ //Rotate
                final double SPEED=-0.4;
                motor1.setPower(-SPEED);
                motor2.setPower(SPEED);
                motor3.setPower(SPEED);
                motor4.setPower(-SPEED);
            } else {
                driveAngle(0,0);
                task2 = true;
                totx = tottotx;
            }
            telemetry.addData("Task","Turning... At "+compass.getDirection()+" and moving towards "+ninetydeg);
        } else if(!task3){
            if(ultra.getUltrasonicLevel()>21){
                driveRight();
            } else {
                driveAngle(0,0);
                task3 = true;
            }
        }

        else if(!task4){
            try {
                detectColor();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            ninetydeg = compass.getDirection();
            task4 = true;
        }

        else if (!task5) {
            if (ultra.getUltrasonicLevel() < 28) {
                driveLeft();
            }

            else {
                driveAngle(0,0);
                totx = tottotx;
                task5 = true;
            }
        }

        else if (!task6){
            if(compass.getDirection()>ninetydeg-180){ //Rotate
                final double SPEED=-0.4;
                motor1.setPower(-SPEED);
                motor2.setPower(SPEED);
                motor3.setPower(SPEED);
                motor4.setPower(-SPEED);
            } else {
                driveAngle(0, 0);
                task6 = true;
                totx = tottotx;
            }
        }

        else if (!task7){
            try {
                hardAutoShooter();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            task7 = true;
        }
        else {
            telemetry.addData("Task","done");
            telemetry.addData("Ultra-",ultra.getUltrasonicLevel());
            telemetry.update();
        }

    }
}

