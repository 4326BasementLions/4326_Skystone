package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(name = "RED - TrueFoundation NOPID 1.7", group = "Iterative Opmode")

public class RedTouchFoundationMachineNOPID extends OpMode {
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

    driveState turnRight;
    driveState turnLeft;
    driveState slightFwd;
    driveState lineUP;
    driveState approachFoundation;
    GyroTurnCWByPID faceFoundation;


    OnlyClaspState letGoGo;

    GyroTurnCCWByPID faceBridge;

    //OnlyClaspState getLetGoGo;
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

        foundation = hardwareMap.servo.get("block");
        //foundationRight = hardwareMap.servo.get("foundation_right");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);




        lineUP  = new driveState(-20, .5, motors, "backward");
        firsTurn = new GyroTurnCCWByPID(84, .5, motors, imu);

        approachFoundation = new driveState(15, .5,  motors, "forward");
        approach = new buttonDriveState(motors, .5, touchSensor, "f"); //drives forward until the button is pressed



        turnRight = new driveState(3, .6 , motors, "turnRight");
        slightFwd = new driveState(2,.3,motors, "forward");
        grabbbygrab = new OnlyClaspState( foundation, 3, 1); //grabby grabs the foundation

        backUp = new driveState(-50, .7, motors, "backward"); //drives back into parking area
        turnLeft = new driveState(45, .6 , motors, "turnRight");



//
        letGoGo = new OnlyClaspState( foundation, 3, 0); //lets go of the foundation
//
//        faceBridge = new GyroTurnCCWByPID(90, .5, motors, imu); //faces bridge
//.
        finals = new ColorSenseStopState(motors, colorSensor, "red", .5, "backward"); //drives and parks under the bridge


        //set up the sequence
        lineUP.setNextState(firsTurn);
        firsTurn.setNextState(approachFoundation);
        approachFoundation.setNextState(approach);
        approach.setNextState(turnRight);
        turnRight.setNextState(grabbbygrab);
        grabbbygrab.setNextState(backUp);
        backUp.setNextState(turnLeft);
        turnLeft.setNextState(letGoGo);
        letGoGo.setNextState(finals);
//        letGoGo.setNextState(faceBridge);
//        faceBridge.setNextState(finals);
        finals.setNextState(null);

        //yay!






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
