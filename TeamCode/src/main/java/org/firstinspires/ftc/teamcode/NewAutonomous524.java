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

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Team 524 Autonomous", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class NewAutonomous524 extends MecanumOpMode {
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
    private double[] acc, vel, pos, output, setpos; //acceleration (WITHOUT g), velocity, position, output (scaled voltage)
    private double[] kp, kd; //output (scaled voltage)
    private double[] accl, magn, g, b; //raw accelerometer and magnetometer values, calibration values (gravity, south/north) (NEEDS NEW DOUBLE???)
    private double[] velo, posi;
    private double[] angvel, angpos;

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

    public NewAutonomous524() {
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        acc = new double[3]; //stores x, y and z accelerations from acc after removing gravity
        vel = new double[3]; //stores x, y and THETA velocities (from acc and mag)
        angvel = new double[3]; //stores x, y and THETA velocities (from acc and mag)
        pos = new double[3]; //stores x, y and theta positions (from acc and mag)
        angpos = new double[3]; //stores x, y and theta positions (from acc and mag)
        output = new double[3];
        setpos = new double[3];
        magn = new double[3];
        accl = new double[3];
        g = new double[3];
        b = new double[3];
        velo = new double[3];
        posi = new double[3];

        //set sample period for sensors
        interval = SensorManager.SENSOR_DELAY_GAME;

        //set initial positions; ask Sagnick for details of what to do (gamepad thingy)
//        pos[0] =;
//        pos[1] =;
//        pos[2] =;
//
//        //set the proportional and derivative constants
//        kp[0] =;
//        kp[1] =;
//        kp[2] =;
//        kd[0] =;
//        kd[1] =;
//        kd[2] =;

        teamColor = "r";

        //NXT
        accelNXT = hardwareMap.accelerationSensor.get("acc");
        gyroNXT = hardwareMap.gyroSensor.get("gyro");

        shooter = hardwareMap.dcMotor.get("shooter");

        ballKeeper = hardwareMap.servo.get("ballKeeper");
        flicker = hardwareMap.servo.get("flicker");
        flicker.setPosition(0.55);
        ballKeeper.setPosition(0.0);
        telemetry.addData("Status", "Initialized");
        initAfterRT = runtime.milliseconds();

        coord[0]=0;
        coord[0]=1;
        coord[0]=0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    private boolean done = false;
    @Override
    public void init_loop() {
        int n = 0; //no. of measurements of b[] and g[]
        //keep measuring the accelerometer and magnetometer until driver hits PLAY
        //make sure robot is stationary and level
        //the last value before break is used for calibration

            /*
            *
            * Code to read the accelerometer and magnetometer goes here
            * Raw acc values are stored in g[]
            * Raw magnetometer values are stored in b[]
            *
            * */

            /*
            * Code to average many (~100) g[] values and store in g[] (using +=)
            * */

        //set this time to 3-4 seconds
        if(!done){
            accl[0] += accX;
            accl[1] += accY;
            accl[2] += accZ;


            n++;
        }
        /*
        THIS PART LOOKS SKETCHY (look carefully at the !done statements)
        */
        if (getRuntime() >= initAfterRT + 3000 && !done) {
            accl[0] = accl[0] / n;
            accl[1] = accl[1] / n;
            accl[2] = accl[2] / n;
            magn[0] = magn[0] / n;
            magn[1] = magn[1] / n;
            magn[2] = magn[2] / n;
                /*
                * Telemetry to display "calibration done"
                * */

            g = unitVector(accl);
            b = unitVector(magn);

            done = true;
        }


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
    private boolean task1, task2, task3;

    @Override
    public void loop() {

        if (!task1 )  {
            // if it gets to that position
            if(goToPosition(coord)){
                //set new coord
                /**
                coord[0] = ;
                coord[0] = ;
                coord[0] = ;
                 */
            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    //this is the PD controller
    public double[] output(double[] setpos) {
        for (int i = 0; i <= 2; i++) {
            output[i] = (kp[i] * (setpos[i] - pos[i])) - (kd[i] * vel[i]);
        }
        return output;
    }

    public boolean goToPosition(double[] setpoint){
        setpos[0] = setpoint[0]; //x position
        setpos[1] = setpoint[1]; //y position
        setpos[2] = setpoint[2]; //orientation

        //should it be <= someNumber instead of ==someNumber? (will the code stop when getRuntime()%interval != 0?)
        if (runtime.milliseconds() % interval <= 19) {
            /*
            *
            * Code to read the accelerometer and magnetometer goes here
            * Raw acc values are stored in accl[]
            * Raw magnetometer values are stored in magn[]
            *
            * */

            accl[0] = accX;
            accl[1] = accY;
            accl[2] = accZ;

            angvel[0] = gyroX;
            angvel[1] = gyroY;
            angvel[2] = gyroZ;

            for (int i = 0; i <= 2; i++){
                acc[i] = accl[i] - g[i];
                velo[i] += acc[i]*interval;
                posi[i] += 0.5*acc[i]*interval*interval;
                angpos[i] += angvel[i]*interval;
            }

            //Matrix a = new Matrix(3,5);

            //ONLY A PLACEHOLDER
            double[] o = output(setpos);
            motor1.setPower(o[0]);
        }
        if(threshold(setpos[0],pos[0]) && threshold(setpos[1],pos[1]) && threshold(setpos[2],pos[2])){
            return true;
        }
        return false;
    }
    public boolean threshold(double one, double two){
        final double THRESHOLD = 0.1;
        if(Math.abs(one-two) < THRESHOLD){
            return true;
        }
        return false;
    }
    //use the version below if the one above gets messy
    /*
    public double outx(double[] setx){
        double setX = setx[0];
        return (kp[0]*(setX - pos[0])) - (kd[0]*vel[0]);
    }

    public double outy(double[] sety){
        double setY = sety[1];
        return (kp[1]*(setY - pos[1])) - (kd[1]*vel[1]);
    }

    public double outh(double[] seth){
        double setH = seth[2];
        return (kp[2]*(setH - pos[2])) - (kd[2]*vel[2]);
    }
    */
    public void onSensorChanged(SensorEvent sensorEvent) {
        switch (sensorEvent.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                accX = accelNXT.getAcceleration().xAccel;
                accY = accelNXT.getAcceleration().yAccel;
                accZ = accelNXT.getAcceleration().zAccel;
                gyroX = gyroNXT.rawX();
                gyroY = gyroNXT.rawY();
                gyroZ = gyroNXT.rawZ();
                break;
           /* case Sensor.TYPE_MAGNETIC_FIELD:
                compassX = sensorEvent.values[0];
                compassY = sensorEvent.values[1];
                compassZ = sensorEvent.values[2];
                break;*/
        }
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