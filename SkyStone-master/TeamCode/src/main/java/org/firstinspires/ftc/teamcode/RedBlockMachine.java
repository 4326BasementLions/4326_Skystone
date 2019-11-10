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

    driveState moveALittle;
    GyroTurnCWByPID reposition;

    OnlyClaspState grabBlock;

    OnlyClaspState startGrabBlock;

    ColorSenseStopState bridgeWithBlock;
    driveState goToMiddleBlock;

    adjustPulleyState adjustPulley;
    ColorSensor colorSensor;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    //Setting up the order

    DcMotor pulley;

    public void init(){

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        JustSkyStoneNavigationState okTest;

        //int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        pulley = hardwareMap.dcMotor.get("pulley");
        clasp = hardwareMap.servo.get("clasp");
        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        motors.add(pulley);
        //okTest = new JustSkyStoneNavigationState(motors, "no", cameraMonitorViewId);

        startGrabBlock = new OnlyClaspState(clasp, 2, .75);
        grabBlock = new OnlyClaspState(clasp,1,1);
        adjustPulley = new adjustPulleyState(.25,.5 ,motors, rightHand, leftHand );
        reposition = new GyroTurnCWByPID(6.5, 0.5, motors, imu); //clockwise

        moveALittle = new driveState(6, .5, motors, "left");//added this to move a little

        getOffBlock = new driveState(22, .5 , motors, "right");

        turnFromBlock = new GyroTurnCCWByPID(110, .5, motors, imu);

        moveBackFromBlock = new driveState(18, .5, motors, "backward");

        bridgeWithBlock = new ColorSenseStopState(motors, colorSensor, "red", .225, "backward");

        close = new OpenClosePulleyState(motors, rightHand, leftHand, "close");

        open = new OpenClosePulleyState(motors, rightHand, leftHand, "open");

        moveDown = new movePulleyState(1.0, .5, motors, rightHand, leftHand);

        moveUp = new movePulleyState(1.0, -.5, motors, rightHand, leftHand);


        goToMiddleBlock = new driveState(.3,.5,motors,"forward");




        //Sequence


        approachBlocks.setNextState(turnBlock);
        turnBlock.setNextState(goToMiddleBlock);
        goToMiddleBlock.setNextState(startGrabBlock);
        //strafeToBlock.setNextState(grabBlock);
        startGrabBlock.setNextState(reposition);
        reposition.setNextState(moveALittle);
        moveALittle.setNextState(grabBlock);
        grabBlock.setNextState(getOffBlock);
        getOffBlock.setNextState(adjustPulley);
        adjustPulley.setNextState(bridgeWithBlock);
        bridgeWithBlock.setNextState(null);


    }

    @Override
    public void start(){
        machine = new StateMachine(approachBlocks);

    }
    private StateMachine machine;
    public void loop()  {


        telemetry.addData("state", machine.currentState());
        telemetry.addData("state", machine.currentState());

        telemetry.update();
        machine.update();

    }
}
