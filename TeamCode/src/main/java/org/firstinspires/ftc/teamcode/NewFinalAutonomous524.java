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
import com.qualcomm.robotcore.hardware.DcMotor;
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


    //Phone sensors
    private Sensor magnetometer;
    private Sensor accelerometer;

    private String teamColor;
    private DcMotor shooter;
    private AccelerationSensor accelNXT;
    private GyroSensor gyroNXT;
    private Servo ballKeeper;
    private Servo flicker;
    private double initAfterRT;

    //PID variables
    private double setx, curx, lastx, tottotx, totx, velx, kp, kd, outx, dx, startingEncoderMotor2, startingEncoderMotor4;

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

    public NewFinalAutonomous524() {
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //set sample period for sensors
        interval = SensorManager.SENSOR_DELAY_GAME;
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        teamColor = "r";

        kp = 1.0;
        kd = 0.0;

        //NXT
        accelNXT = hardwareMap.accelerationSensor.get("acc");

        shooter = hardwareMap.dcMotor.get("shooter");

        ballKeeper = hardwareMap.servo.get("ballKeeper");
        flicker = hardwareMap.servo.get("flicker");
        flicker.setPosition(0.55);
        ballKeeper.setPosition(0.0);
        telemetry.addData("Status", "Initialized");
        initAfterRT = runtime.milliseconds();
        startingEncoderMotor2 = motor2.getCurrentPosition();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("startingEncoderMotor2",startingEncoderMotor2);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    private boolean task1 = false;
    private boolean task2, task3;

    @Override
    public void loop() {
        telemetry.addData("encoder 1", motor2.getCurrentPosition()  );
//        driveOneJoystick(gamepad1,"left");
        if (!task1)  {
            totx = tottotx;
            goToPosition(1);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public void goToPosition(double setpointx){
        setx=setpointx; //x position
        final double DIAMETER = 2;
        tottotx = ((((motor2.getCurrentPosition()-startingEncoderMotor2)))/1426) * DIAMETER;

        //should it be <= someNumber instead of ==someNumber? (will the code stop when getRuntime()%interval != 0?)
        if (runtime.milliseconds() % interval <= 19) {
            curx = tottotx - totx;

            velx = (kd*(curx - lastx))/interval;

            outx = (kp*(setx-curx)) - velx;

            if (outx >= 1){
                outx = 1;
            }

            if (outx <= -1){
                outx = -1;
            }

            driveAngle(Math.PI/2 , outx);
        }
    }

    public void turn(double angle){

    }

    public double dotProduct(double[] vector1, double[] vector2){
        return (vector1[0]*vector2[0]) + (vector1[1]*vector2[1]) + (vector1[2]*vector2[2]);
    }

    public double[] crossProduct(double[] vector1, double[] vector2){
        double[] cp = new double[3];

        cp[0] = (vector1[1]*vector2[2]) - (vector1[2]*vector2[1]);
        cp[1] = (vector1[2]*vector2[0]) - (vector1[0]*vector2[2]);
        cp[2] = (vector1[0]*vector2[1]) - (vector1[1]*vector2[0]);

        return cp;
    }

    public double[] unitVector(double[] vector1){
        double[] uv = new double[3];

        for (int i = 0; i <=2; i++){
            uv[i] = vector1[i]/dotProduct(vector1, vector1);
        }

        return uv;
    }
}