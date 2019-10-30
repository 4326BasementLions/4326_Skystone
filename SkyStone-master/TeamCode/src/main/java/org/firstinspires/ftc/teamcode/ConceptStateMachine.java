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





@Autonomous(name="State Machine Test 2.76", group="Iterative Opmode")
public class ConceptStateMachine extends OpMode
{
//test comment!ahs
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo clasp;

    ColorSensor colorSensor;


    //MattBran mech materials
    DcMotor pulley;

    Servo leftHand;

    Servo rightHand;

    public timeState Test;// new timeState(2,5, motors, "forward");;
    DriveAvoidPIDState Testy;

    GyroTurnCWByPID Turn90;
    adjustPulleyState adjustPulley;

    BNO055IMU imu;


    colorMoveState parkUnderBridge;
    ColorSenseStopState parkUnderBridge2;

    driveState driveToFoundation;
    driveState straighten;
    driveState getOffWall;

    timeState driveBack;

    timeState miniDrive2;

    OnlyClaspState foundationClasp;
    OnlyClaspState releaseClasp;



    GyroTurnCCWByPID turnRightAngle;


    driveState dragFoundationIn;


    driveState approachBlocks;

     GyroTurnCCWByPID turnBlock;

     OnlyClaspState grabBlock;

     driveState strafeToBlock;

    //driveState confirmclasp;
    driveState getOffBlock;
driveState moveBackFromBlock;
    GyroTurnCCWByPID turnFromBlock;
    OpenClosePulleyState close;
    OpenClosePulleyState open;

    movePulleyState moveDown;
    movePulleyState moveUp;

    driveState moveALittle;
    GyroTurnCWByPID reposition;


    ElapsedTime mRuntime = new ElapsedTime();





    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        clasp = hardwareMap.servo.get("clasp");

        leftHand = hardwareMap.servo.get("left");
        rightHand = hardwareMap.servo.get("right");

        pulley = hardwareMap.dcMotor.get("pulley");



        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        motors.add(pulley);

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
        driveToFoundation = new driveState(50, .4, motors, "forward");

        adjustPulley = new adjustPulleyState(.2,.5 ,motors, rightHand, leftHand );

        foundationClasp = new OnlyClaspState( clasp, 2,  .8);


        parkUnderBridge2 = new ColorSenseStopState(motors, colorSensor, "red", .225, "forward");

        turnRightAngle = new GyroTurnCCWByPID(90, .5, motors, imu);
        driveBack = new timeState(5, .5, motors, "backward");

        releaseClasp = new OnlyClaspState( clasp, 2, 0 );
        straighten = new driveState(5,.5, motors, "right");
        getOffWall = new driveState(2, .5 , motors, "left");
        dragFoundationIn = new driveState(5, .5 , motors, "right");


        //GET BLOCKS

        approachBlocks = new driveState(22,.5,motors,"forward");

        turnBlock = new GyroTurnCCWByPID(70,.5,motors,imu); //counterclockwise

        strafeToBlock = new driveState(4.75,.5,motors, "left");
      //  confirmclasp = new driveState(2,.5,motors, "left");

        grabBlock = new OnlyClaspState(clasp,3,1);

       reposition = new GyroTurnCWByPID(6.5, 0.5, motors, imu); //clockwise
        moveALittle = new driveState(5, .5, motors, "left");//added this to move a little




        getOffBlock = new driveState(20, .5 , motors, "right");

turnFromBlock = new GyroTurnCCWByPID(110, .5, motors, imu);

moveBackFromBlock = new driveState(20, .5, motors, "backward");

        close = new OpenClosePulleyState(motors, rightHand, leftHand, "close");

        open = new OpenClosePulleyState(motors, rightHand, leftHand, "open");

        moveDown = new movePulleyState(1.0, .5, motors, rightHand, leftHand);

        moveUp = new movePulleyState(1.0, -.5, motors, rightHand, leftHand);


        //HUGLIFT








//open.setNextState(mo);
moveDown.setNextState(close);
close.setNextState(moveUp);
moveUp.setNextState(open);
open.setNextState(null);

//        //Setting up the order
//        adjustPulley.setNextState(driveToFoundation);
//        driveToFoundation.setNextState(adjustPulley);
//        adjustPulley.setNextState(foundationClasp);
//       // foundationClasp.setNextState(driveBack);
//        driveBack.setNextState(dragFoundationIn);
//        dragFoundationIn.setNextState(releaseClasp);
//        releaseClasp.setNextState(straighten);
//        straighten.setNextState(getOffWall);
//        getOffWall.setNextState(parkUnderBridge2);
//
        approachBlocks.setNextState(turnBlock);
        turnBlock.setNextState(strafeToBlock);
        strafeToBlock.setNextState(grabBlock);
        grabBlock.setNextState(reposition);
       reposition.setNextState(moveALittle);
        moveALittle.setNextState(getOffBlock);
        getOffBlock.setNextState(adjustPulley);
        adjustPulley.setNextState(moveBackFromBlock);
        moveBackFromBlock.setNextState(null);
    //    turnFromBlock.setNextState(null);
        //If


        //confirmclasp.setNextState(null);
////        getOffWall.setNextState(null);
//        close.setNextState(foundationClasp);
//
//        foundationClasp.setNextState(open);






    }


    @Override
    public void start(){
        machine = new StateMachine(approachBlocks);
        while (machine.currentState().equals(grabBlock)){
//            mRuntime.reset();
//            if(mRuntim){
//
//            }
            wait(1);
            //break;
        }

    }



    @Override
    public void loop()  {


        telemetry.addData("state", machine.currentState());
        telemetry.addData("state", machine.currentState());

        telemetry.update();
         machine.update();

    }

   private StateMachine machine;

    @Override
    public void stop() {
    }

    public void wait(int time) {
        try {
            Thread.sleep(time * 100);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
