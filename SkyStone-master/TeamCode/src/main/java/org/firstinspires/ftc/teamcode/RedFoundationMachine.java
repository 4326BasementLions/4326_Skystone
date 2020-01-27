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

@Disabled
@Autonomous(name="RedFoundation", group="Iterative Opmode")
public class RedFoundationMachine extends OpMode{


    Servo leftHand;
    Servo rightHand;
    Servo clasp;
    BNO055IMU imu;

    adjustPulleyState adjustPulley;
  //  driveState driveToFoundation;
    driveState Step1;
    distanceState driveToFoundation;
    distanceState strafeAwayFoundation;
    driveState straighten;
    driveState getOffWall;
    timeState driveBack;
    OnlyClaspState foundationClasp;
    OnlyClaspState releaseClasp;
    driveState dragFoundationIn;
    ColorSenseStopState parkUnderBridge2;
    GyroTurnCCWByPID turnRightAngle;
driveState littleLeft;
driveState littleForward;
GyroTurnCWByPID turnLeftAngle;
driveState strafeToFoundation;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    ColorSensor colorSensor;
    /*ModernRoboticsI2cRangeSensor*/ DistanceSensor distance;
    /*ModernRoboticsI2cRangeSensor*/ DistanceSensor distance2;

    //Setting up the order
            DcMotor pulley;


        public void init(){


            colorSensor = hardwareMap.colorSensor.get("colorSensor");
            rightHand = hardwareMap.servo.get("right");
            leftHand = hardwareMap.servo.get("left");
            rightFront = hardwareMap.dcMotor.get("right front");
            leftFront = hardwareMap.dcMotor.get("left front");
            rightBack = hardwareMap.dcMotor.get("right back");
            leftBack = hardwareMap.dcMotor.get("left back");
            pulley = hardwareMap.dcMotor.get("pulley");
            clasp = hardwareMap.servo.get("clasp");
            distance = hardwareMap.get(DistanceSensor.class, "distance");//(ModernRoboticsI2cRangeSensor.class, "distance");
            distance2 = hardwareMap.get(DistanceSensor.class, "distance2");//(ModernRoboticsI2cRangeSensor.class, "distance2");

            ArrayList<DcMotor> motors = new ArrayList<>();
            motors.add(rightFront);
            motors.add(leftFront);
            motors.add(rightBack);
            motors.add(leftBack);
            motors.add(pulley);

            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;

            imu = hardwareMap.get(BNO055IMU.class, "imu"); // lmao hardware what a joke

            imu.initialize(parameters);

            //driveToFoundation = new driveState(47, .4, motors, "forward");

            //plan: drive to foundation, move to side of foundation, drive up to the side of the foundation, grab, drag back to corner, park


            Step1 = new driveState(22, .25, motors, "forward");
            driveToFoundation = new distanceState(distance,.5, 12, motors, "forward", "fc"); //drives to the foundation
            turnRightAngle = new GyroTurnCCWByPID(90, .5, motors, imu);
//
            //turnLeftAngle = new GyroTurnCWByPID(90, .5, motors, imu);
            strafeToFoundation = new driveState(4,.5, motors, "left");

//            strafeAwayFoundation = new distanceState(distance,.5, 1.5, motors, "left", "cf"); //strafes until it can no longer see the foundation (finds the edge)
//
//            littleLeft = new driveState(4.5, .5, motors, "left"); //readjusts bot's position – since sensor is not on the corner of the bot
              littleForward = new driveState(5, .5, motors, "backward");
//            adjustPulley = new adjustPulleyState(.25,-.5 ,motors, rightHand, leftHand ); //adjusts pulley mechanism to lower so we can go under the bridge

            foundationClasp = new OnlyClaspState(clasp, 2,  1.2); //grabs foundation


//            parkUnderBridge2 = new ColorSenseStopState(motors, colorSensor, "red", .225, "forward"); //parks
//
//            turnRightAngle = new GyroTurnCCWByPID(90, .5, motors, imu);
//            driveBack = new timeState(5, .5, motors, "backward");
//
//            releaseClasp = new OnlyClaspState(clasp, 2, 0 );
//            straighten = new driveState(5,.5, motors, "right");
//            getOffWall = new driveState(2, .5 , motors, "left");
            dragFoundationIn = new driveState(30, .8 , motors, "right");



          //Sequence


           Step1.setNextState(driveToFoundation);
            driveToFoundation.setNextState(turnRightAngle);
            turnRightAngle.setNextState(strafeToFoundation);
            strafeToFoundation.setNextState(littleForward);
            littleForward.setNextState(foundationClasp);
            foundationClasp.setNextState(dragFoundationIn);


//            strafeAwayFoundation.setNextState(littleLeft);
//            littleLeft.setNextState(littleForward);
//            // adjustPulley.setNextState(foundationClasp);
//            littleForward.setNextState(foundationClasp);
//            foundationClasp.setNextState(driveBack);
//            driveBack.setNextState(dragFoundationIn);
//            dragFoundationIn.setNextState(releaseClasp);
//            releaseClasp.setNextState(straighten);
//            straighten.setNextState(getOffWall);
//            //adjustPulley.setNextState(getOffWall);
//            getOffWall.setNextState(parkUnderBridge2);
//            parkUnderBridge2.setNextState(adjustPulley);


        }

    @Override
    public void start(){
        machine = new StateMachine(Step1);

    }
    private StateMachine machine;
    public void loop()  {


        telemetry.addData("state", machine.currentState());
        telemetry.addData("state", machine.currentState());

        telemetry.update();
        machine.update();

    }
}
