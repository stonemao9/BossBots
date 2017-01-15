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
    }

    @Override
    public void init_loop() {
        try {
            fou = hardwareMap.appContext.openFileOutput("data_new.txt", MODE_APPEND);
            outputStreamWriter = new OutputStreamWriter(fou);
            telemetry.addData("created", "output stream writer");
        } catch (FileNotFoundException e) {
            Log.e("Error", "INIT-tried creating fileoutputstream" + e.toString());
            telemetry.addData("Error", e);
        }
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        updateSensor(true);
        driveOneJoystick(gamepad1, "left");
        telemetry.addData("accX", accX);
        telemetry.addData("accY", accY);
        telemetry.addData("accZ", accZ);
        output += runtime.seconds()+","+accX + "," + accY + "," + accZ + "\n";
    }

    public void stop() {
        FileWriter writer;
        File out = new File("sample.txt");
        Log.d("FTC STOP","writing file");
        try {
            writer = new FileWriter(out,true);
            writer.write(output);
            Log.d("FTC WRITER","WRITING");
            writer.flush();
            writer.close();
            Log.w("FTC Done",out.getPath());
        } catch (IOException e) {
            e.printStackTrace();
            Log.e("FTC ERROR", output);
        }

    }

}
