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
    ColorSenseStopState bridgeWithBlock;
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

     OnlyClaspState startGrabBlock;

     driveState strafeToBlock;

     driveState goToMiddleBlock;

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

    JustSkyStoneNavigationState okTest;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            " ATfbkf//////AAABmSdKrt64X0UDrXZRbiIdwVl9iuhdq1WQN1irJAz1O/XAe4vAgTnNCQsLzqtENwAZjOfmIvzpWoO8CD4VW6uZ6gGSYAv8gLSG4Ew+HLqDbKrN+gyhJPkxwiKDFXIHWeSNuGh3UUSKGj++8ggR9vYFTyLqXpvy2uwI+z66wWL3aPUU5KjK0N8oy5+IyddBgKGDHw2QacCqKJvMuL+VOOPNYdwKC3nQ+caRIS4gsJQwQ3FZrgY/oHgfse+vLRdoBKfhV2Pl6d2kqphlXivEWaPcvkOrpkkJvqR7aYwvkkO6Aqlph6YdLRp6okEauD6zly8s4rUqoCKmOd4cEx8TfamSqg/jhc4eRbN0koLdkOWL53nG";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    ElapsedTime mRuntime = new ElapsedTime();

     int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//;
      public CameraDevice cam;//camera!
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    @Override
    public void init() {

//        rightFront = hardwareMap.dcMotor.get("right front");
//        leftFront = hardwareMap.dcMotor.get("left front");
//        rightBack = hardwareMap.dcMotor.get("right back");
//        leftBack = hardwareMap.dcMotor.get("left back");
//
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//
//        colorSensor = hardwareMap.colorSensor.get("colorSensor");
//
//        clasp = hardwareMap.servo.get("clasp");
//
//        leftHand = hardwareMap.servo.get("left");
//        rightHand = hardwareMap.servo.get("right");
//
//        pulley = hardwareMap.dcMotor.get("pulley");
//
//
//
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.setDirection(DcMotor.Direction.REVERSE);


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
//        rightFront.setPower(0);
//        leftFront.setPower(0);
//        rightBack.setPower(0);
//        leftBack.setPower(0);

        //State Declarations
        driveToFoundation = new driveState(50, .4, motors, "forward");

        adjustPulley = new adjustPulleyState(.25,.5 ,motors, rightHand, leftHand );

        foundationClasp = new OnlyClaspState( clasp, 2,  .8);


        parkUnderBridge2 = new ColorSenseStopState(motors, colorSensor, "red", .225, "forward");

        turnRightAngle = new GyroTurnCCWByPID(90, .5, motors, imu);
        driveBack = new timeState(5, .5, motors, "backward");

        releaseClasp = new OnlyClaspState( clasp, 2, 0 );
        straighten = new driveState(5,.5, motors, "right");
        getOffWall = new driveState(2, .5 , motors, "left");
        dragFoundationIn = new driveState(5, .5 , motors, "right");


        //GET BLOCKS

        approachBlocks = new driveState(24.4,.5,motors,"forward");

        turnBlock = new GyroTurnCCWByPID(90,.5,motors,imu); //counterclockwise

        strafeToBlock = new driveState(4.1,.5,motors, "left");
      //  confirmclasp = new driveState(2,.5,motors, "left");

       startGrabBlock = new OnlyClaspState(clasp, 2, .75);
        grabBlock = new OnlyClaspState(clasp,1,1);

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


        //HUGLIFT
          okTest = new JustSkyStoneNavigationState(motors, "no", cameraMonitorViewId);






okTest.setNextState(null);
//open.setNextState(mo);
/*moveDown.setNextState(close);
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
        turnBlock.setNextState(goToMiddleBlock);
        goToMiddleBlock.setNextState(startGrabBlock);
        //strafeToBlock.setNextState(grabBlock);
        startGrabBlock.setNextState(reposition);
        reposition.setNextState(moveALittle);
        moveALittle.setNextState(grabBlock);
        grabBlock.setNextState(getOffBlock);
        getOffBlock.setNextState(adjustPulley);
        adjustPulley.setNextState(bridgeWithBlock);
        bridgeWithBlock.setNextState(null);*/
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
        machine = new StateMachine(okTest);
//        while (machine.currentState().equals(okTest)){
////            mRuntime.reset();
////            if(mRuntim){
////
////            }
//            wait(1);
//            //break;
//        }

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
