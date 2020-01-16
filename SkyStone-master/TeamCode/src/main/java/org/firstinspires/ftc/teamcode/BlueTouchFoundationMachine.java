package org.firstinspires.ftc.teamcode;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;

@Autonomous(name = "TrueBlueFoundation", group = "Iterative Opmode")

public class BlueTouchFoundationMachine extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo foundationLeft;

    Servo foundationRight;

    DigitalChannel touchSensor;

    ColorSensor colorSensor;

    BNO055IMU imu;

    private StateMachine machine;

    //cool

    buttonDriveState approach;

    DoubleClaspState grabbbygrab;

    driveState backUp;

    DoubleClaspState letGoGo;

    GyroTurnCWByPID faceBridge;

    ColorSenseStopState finals;

    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftBack = hardwareMap.dcMotor.get("left_back");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        touchSensor = hardwareMap.get(DigitalChannel.class, "sensor_digital"); //configure as digital device, not a REV touch sensor!!!
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        foundationLeft = hardwareMap.servo.get("foundation_left");
        foundationRight = hardwareMap.servo.get("foundation_right");


        //State set up

        approach = new buttonDriveState(motors, .5, touchSensor); //drives forward until the button is pressed

        grabbbygrab = new DoubleClaspState(foundationLeft, foundationRight, 1, 0); //grabby grabs the foundation

        backUp = new driveState(30, .5, motors, "backward"); //drives back into parking area

        letGoGo = new DoubleClaspState(foundationLeft, foundationRight, 1, 1); //lets go of the foundation

        faceBridge = new GyroTurnCWByPID(90, .5, motors, imu); //faces bridge

        finals = new ColorSenseStopState(motors, colorSensor, "blue", .5, "forward"); //drives and parks under the bridge


        //set up the sequence

        approach.setNextState(grabbbygrab);
        grabbbygrab.setNextState(backUp);
        backUp.setNextState(letGoGo);
        letGoGo.setNextState(faceBridge);
        faceBridge.setNextState(finals);

        //yay!






    }

    @Override
    public void start(){
        machine = new StateMachine(approach);
        machine.update();
    }

    @Override
    public void loop(){
        machine.update();
        telemetry.update();
    }


}
