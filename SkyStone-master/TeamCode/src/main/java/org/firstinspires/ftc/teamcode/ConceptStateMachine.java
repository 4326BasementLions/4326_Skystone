package org.firstinspires.ftc.teamcode;
//This is a test for state machines, iterative opmode

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="State Machine Test 2.05", group="Iterative Opmode")
public class ConceptStateMachine extends OpMode
{
//test comment!
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo clasp;

    ColorSensor colorSensor;

    public timeState Test;// new timeState(2,5, motors, "forward");;
    DriveAvoidPIDState Testy;

    GyroTurnCWByPID Turn90;


    BNO055IMU imu;


    colorMoveState parkUnderBridge;
    ColorSenseStopState parkUnderBridge2;

    timeState miniDrive;

    timeState miniDrive2;

    ClaspState justClasp;

    ClaspState grabbyGrab;

    GyroTurnCCWByPID turnRightAngle;

    driveState moveOnce;



    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        clasp = hardwareMap.servo.get("clasp");



        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        // Set all motors to zero power
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);


        //State Declarations
       miniDrive = new timeState(5, .5, motors, "forward");
        //miniDrive2 = new timeState(2, .5, motors, "backward");

        moveOnce = new driveState(30, .5, motors, "forwards");

        justClasp = new ClaspState(motors, clasp, 2, "forward", 0, .4);
        grabbyGrab = new ClaspState(motors, clasp, 5, "backward", .5, .4);
        parkUnderBridge2 = new ColorSenseStopState(motors, colorSensor, "red", .15, "forward");
        //Turn90 = new GyroTurnCWByPID(270, 5, motors, imu);
        turnRightAngle = new GyroTurnCCWByPID(90, .5, motors, imu);
       // grabbyGrab = new ClaspState(motors, clasp, 2);

        //Setting up the order
        miniDrive.setNextState(justClasp);
        moveOnce.setNextState(justClasp);
        justClasp.setNextState(grabbyGrab);
        grabbyGrab.setNextState(null);
        turnRightAngle.setNextState(parkUnderBridge2);
        parkUnderBridge2.setNextState(null);
        //grabbyGrab.setNextState(null);





    }


    @Override
    public void start(){
        machine = new StateMachine(moveOnce);

    }


    @Override
    public void loop()  {


        //telemetry.addData("correction", Testy.getTelemetry());
       // telemetry.addData("color", parkUnderBridge.getColor());

        telemetry.addData("redVal", parkUnderBridge2.getColor());

        telemetry.update();
         machine.update();

    }

    private StateMachine machine;

    @Override
    public void stop() {
    }}
