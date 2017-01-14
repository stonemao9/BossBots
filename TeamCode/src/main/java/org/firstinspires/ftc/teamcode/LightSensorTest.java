package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import android.content.*;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.util.Log;

import static android.content.Context.MODE_APPEND;

@TeleOp(name="Template: Iterative OpMode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled

public class LightSensorTest extends MecanumOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    FileOutputStream fou;
    OutputStreamWriter outputStreamWriter;

    private String output="";

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        accSense = hardwareMap.accelerationSensor.get("acc");
        gyroSense = hardwareMap.gyroSensor.get("gyro");

        try {
            fou = hardwareMap.appContext.openFileOutput("data.txt", MODE_APPEND);
            outputStreamWriter = new OutputStreamWriter(fou);
        } catch (FileNotFoundException e) {
            Log.e("Error", "INIT-tried creating fileoutputstream"+e.toString());
        }
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        output += accX+","+accY+","+accZ+"\n";
        write(output);
    }
    public void stop() {
        try {
            outputStreamWriter.close();
        }
        catch (IOException e) {
            Log.e("FTC-Exception-Stop", "File write failed: " + e.toString());
        }
    }
    public void write(String data){
        try {
            outputStreamWriter.write(data);
        }
        catch (IOException e) {
            Log.e("FTC-Exception-write()", "File write failed: " + e.toString());
        }
    }

}
