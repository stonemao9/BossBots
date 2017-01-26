package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import android.content.*;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.os.Environment;
import android.util.Log;
import android.widget.Toast;

import static android.content.Context.MODE_APPEND;
import static android.content.Context.MODE_PRIVATE;

@TeleOp(name = "Template: Iterative OpMode", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled

public class LightSensorTest extends MecanumOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    FileOutputStream fou;
    OutputStreamWriter outputStreamWriter;
    FileWriter writer;
    File out;
    private String output = "";

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);
        accSense = hardwareMap.accelerationSensor.get("acc");
        gyroSense = hardwareMap.gyroSensor.get("gyro");

        try {
            out = new File("/sdcard/sample.txt");
            Log.d("FTC-init", "File class created");
            out.createNewFile();
            Log.d("FTC-init", "File class created new file");
            fou = new FileOutputStream(out);
            Log.d("FTC-init", "FileOutputStream created");
            outputStreamWriter = new OutputStreamWriter(fou);
            Log.d("FTC-init", "OutputStreamWriter Created");
        } catch (IOException e) {
            Log.e("FTC-INIT", e.toString());
        }
    }

    @Override
    public void init_loop() {
        updateSensor(true);
        telemetry.addData("accX", accX);
        telemetry.addData("accY", accY);
        telemetry.addData("accZ", accZ);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        telemetry.addData("Status", "Running: " + runtime.toString());
        updateSensor(true);
        if (runtime.milliseconds() < 2000) {
            driveAngle(Math.PI/2,1);
            telemetry.addData("accX", accX);
            telemetry.addData("accY", accY);
            telemetry.addData("accZ", accZ);
            output += runtime.seconds() + "," + accX + "," + accY + "," + accZ + "\n";
        } else if(runtime.milliseconds()<3000) {
            driveAngle(Math.PI/2,0);
        } else {
            stop();
        }
    }

    public void stop() {
        Log.d("FTC STOP", "writing file");
        try {
            outputStreamWriter.append(output);
            Log.d("FTC-stop", "OutputStreamWriter successfully appended output");
            outputStreamWriter.close();
            Log.d("FTC-stop", "OutputStream Writer closed");
            fou.close();
            Log.d("FTC-stop", "Fou closed");
            Log.w("FTC Done", out.getPath());
        } catch (IOException e) {
            e.printStackTrace();
            Log.e("Error-FTC", e.toString());
        }
    }

}
