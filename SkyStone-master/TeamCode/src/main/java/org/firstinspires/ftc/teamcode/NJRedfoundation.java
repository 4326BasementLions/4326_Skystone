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

@Autonomous(name = "RED - NJ foundation", group = "Iterative Opmode")

public class NJRedfoundation extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo foundation;
    Servo block;

    Servo skystone;

    // Servo foundationRight;

    DigitalChannel touchSensor;

    ColorSensor colorSensor;

    BNO055IMU imu;

    private StateMachine machine;

    //cool
    driveState lineUP;
    driveState approachFoundation;
    GyroTurnCCWByPID firsTurn;
    GyroTurnCCWByPID secondTurn;



    buttonDriveState approach;



    DoubleClaspState getFoundation;
    DoubleClaspState ungetFoundation;


    driveState goBack; //target 30
    driveState pushFoundationIn;
    driveState approachSkybridge; //target 30

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

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE); //leftFront
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        touchSensor = hardwareMap.get(DigitalChannel.class, "sensor_digital"); //configure as digital device, not a REV touch sensor!!!
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        foundation = hardwareMap.servo.get("foundation");

        block = hardwareMap.servo.get("block");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        //foundationRight = hardwareMap.servo.get("foundation_right");

        lineUP  = new driveState(-17, .5, motors, "backward");
        firsTurn = new GyroTurnCCWByPID(90, .5, motors, imu);

        approachFoundation = new driveState(13, .5,  motors, "forward");
        approach = new buttonDriveState(motors, .5, touchSensor, "f"); //drives forward until the button is pressed

        getFoundation = new DoubleClaspState(foundation, block, 3, 0,1);
        goBack = new driveState(-15, .5, motors, "backward");
        secondTurn = new GyroTurnCCWByPID(90, .5, motors, imu);
        ungetFoundation = new DoubleClaspState(foundation, block, 3, 1,0);// comment
        pushFoundationIn = new driveState(15, .5 ,motors, "forward"); // comment
        approachSkybridge = new driveState(-30, .5, motors, "backward"); // cpomment
        finals = new ColorSenseStopState(motors, colorSensor, "red", .5, "backward"); //drives and parks under the bridge

        //State set up


        //yay!
        lineUP.setNextState(firsTurn);
        firsTurn.setNextState(approachFoundation);
        approachFoundation.setNextState(approach);
        approach.setNextState(getFoundation);
        getFoundation.setNextState(goBack);
        goBack.setNextState(secondTurn);
        secondTurn.setNextState(ungetFoundation);
        ungetFoundation.setNextState(pushFoundationIn);
        pushFoundationIn.setNextState(approachSkybridge);
        approachSkybridge.setNextState(finals);



    }

    @Override
    public void start(){
        machine = new StateMachine(lineUP);
        machine.update();
    }

    @Override
    public void loop(){
        machine.update();
        telemetry.update();
    }


}
