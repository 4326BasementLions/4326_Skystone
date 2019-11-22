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


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.Servo;

import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.*;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="RedBlockMachine", group="Iterative Opmode")
public class RedBlockMachine extends OpMode{

    Servo leftHand;
    Servo rightHand;
    Servo clasp;
    BNO055IMU imu;
    driveState approachBlocks;
    GyroTurnCCWByPID turnBlock;
    driveState getOffBlock;
    driveState moveBackFromBlock;
    GyroTurnCCWByPID turnFromBlock;
    OpenClosePulleyState close;
    OpenClosePulleyState open;

    movePulleyState moveDown;
    movePulleyState moveUp;

    distanceState moveALittle;
  //  GyroTurnCWByPID reposition;

    OnlyClaspState grabBlock;

    OnlyClaspState startGrabBlock;

    ColorSenseStopState bridgeWithBlock;
   // driveState goToMiddleBlock;

    adjustPulleyState adjustPulley;
    ColorSensor colorSensor;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    driveState leaveCorner;


    DistanceSensor distance;
    //GyroTurnCCWByPID turn1;
    distanceState approach;

    //Setting up the order

    DcMotor pulley;

    public void init(){



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        JustSkyStoneNavigationState okTest;

        //int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightHand = hardwareMap.servo.get("right");
        leftHand = hardwareMap.servo.get("left");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        pulley = hardwareMap.dcMotor.get("pulley");
        clasp = hardwareMap.servo.get("clasp");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        motors.add(pulley);
        //okTest = new JustSkyStoneNavigationState(motors, "no", cameraMonitorViewId);
        //turn1 = new GyroTurnCCWByPID(90, 0.5, motors, imu);

        approach = new distanceState(distance, 0.5, 1 /*need to measure how far away from the blocks to stop*/, motors, "forward", "fc");

        approachBlocks = new driveState(24.1,.5,motors,"forward");

        turnBlock = new GyroTurnCCWByPID(90,.5,motors,imu); //counterclockwise

        //strafeToBlock = new driveState(4.1,.5,motors, "left");

      //  goToMiddleBlock = new driveState(.3,.5,motors,"forward");


        startGrabBlock = new OnlyClaspState(clasp, 2, .75);

    //    reposition = new GyroTurnCWByPID(6.5, 0.5, motors, imu); //clockwise

        moveALittle = new distanceState(distance, .5, 0.5, motors, "right", "fc");//added this to move a little


        grabBlock = new OnlyClaspState(clasp,1,1);

        getOffBlock = new driveState(22, .5 , motors, "right");

        adjustPulley = new adjustPulleyState(.25,-.5 ,motors, rightHand, leftHand );

        bridgeWithBlock = new ColorSenseStopState(motors, colorSensor, "blue", .225, "backward");

        leaveCorner = new driveState(20 /*need to measure how far away from the blocks to stop*/, 0.5, motors, "right");

        //Sequence


        //turn1.setNextState(approach);
        approach.setNextState(startGrabBlock);
     //   goToMiddleBlock.setNextState(startGrabBlock);
        //strafeToBlock.setNextState(grabBlock);
        startGrabBlock.setNextState(moveALittle);
  //      reposition.setNextState(moveALittle);
        moveALittle.setNextState(grabBlock);
        grabBlock.setNextState(getOffBlock);
        getOffBlock.setNextState(adjustPulley);
        adjustPulley.setNextState(bridgeWithBlock);
        bridgeWithBlock.setNextState(null);


    }

    @Override
    public void start(){
        machine = new StateMachine(approach);

    }
    private StateMachine machine;
    public void loop()  {


        telemetry.addData("state", machine.currentState());
        telemetry.addData("state", machine.currentState());

        telemetry.update();
        machine.update();

    }
}
