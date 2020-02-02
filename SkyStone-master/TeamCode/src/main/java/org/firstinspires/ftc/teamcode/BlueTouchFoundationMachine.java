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

@Autonomous(name = "BLUE - TrueFoundation 1.3", group = "Iterative Opmode")

public class BlueTouchFoundationMachine extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo foundation;

    Servo skystone;

    // Servo foundationRight;

    DigitalChannel touchSensor;

    ColorSensor colorSensor;

    BNO055IMU imu;

    private StateMachine machine;

    //cool

    buttonDriveState approach;

    OnlyClaspState grabbbygrab;

    driveState backUp;

    GyroTurnCCWByPID firsTurn;



    OnlyClaspState letGoGo;

    GyroTurnCCWByPID faceBridge;

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

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE); //leftFront
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        touchSensor = hardwareMap.get(DigitalChannel.class, "sensor_digital"); //configure as digital device, not a REV touch sensor!!!
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        foundation = hardwareMap.servo.get("foundation");
        //foundationRight = hardwareMap.servo.get("foundation_right");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        //State set up

        approach = new buttonDriveState(motors, .3, touchSensor,"b"); //drives forward until the button is pressed

        grabbbygrab = new OnlyClaspState( foundation, 3, 1); //grabby grabs the foundation

        backUp = new driveState(30, .7, motors, "forward"); //drives back into parking area

        firsTurn = new GyroTurnCCWByPID(90, .5, motors, imu);


//
//        letGoGo = new OnlyClaspState( foundation, 1, 1); //lets go of the foundation
//
//        faceBridge = new GyroTurnCCWByPID(90, .5, motors, imu); //faces bridge
//
        finals = new ColorSenseStopState(motors, colorSensor, "blue", .5, "forward"); //drives and parks under the bridge


        //set up the sequence

        approach.setNextState(grabbbygrab);
        grabbbygrab.setNextState(backUp);
        backUp.setNextState(firsTurn);
        firsTurn.setNextState(null);
//        letGoGo.setNextState(faceBridge);
//        faceBridge.setNextState(finals);
        finals.setNextState(null);

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
