package org.firstinspires.ftc.teamcode;
//This is a test for state machines, iterative opmode

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


@Autonomous(name="Red foundation2 no park", group="Iterative Opmode")
public class RedFoundationMachineNoParkv2 extends OpMode
{
    //test comment!ahs
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo clasp;

    ColorSensor colorSensor;


    DcMotor pulley;

    Servo leftHand;

    Servo rightHand;

    public timeState Test;// new timeState(2,5, motors, "forward");;
    DriveAvoidPIDState Testy;

    adjustPulleyState adjustPulley;

    BNO055IMU imu;

    driveState driveToFoundation;
    driveState driveToBuildingZone;
    driveState strafeIntoBuildingZone;
    driveState pushFoundation;

    distanceState2 distanceDriveToFoundation;
    OnlyClaspState foundationClasp;
    DistanceSensor distanceSensor;
    OnlyClaspState releaseFoundation;
    ColorSenseStopState parkUnderBridge;



//    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
//    private static final boolean PHONE_IS_PORTRAIT = false  ;
//
//    private static final String VUFORIA_KEY =
//            " ATfbkf//////AAABmSdKrt64X0UDrXZRbiIdwVl9iuhdq1WQN1irJAz1O/XAe4vAgTnNCQsLzqtENwAZjOfmIvzpWoO8CD4VW6uZ6gGSYAv8gLSG4Ew+HLqDbKrN+gyhJPkxwiKDFXIHWeSNuGh3UUSKGj++8ggR9vYFTyLqXpvy2uwI+z66wWL3aPUU5KjK0N8oy5+IyddBgKGDHw2QacCqKJvMuL+VOOPNYdwKC3nQ+caRIS4gsJQwQ3FZrgY/oHgfse+vLRdoBKfhV2Pl6d2kqphlXivEWaPcvkOrpkkJvqR7aYwvkkO6Aqlph6YdLRp6okEauD6zly8s4rUqoCKmOd4cEx8TfamSqg/jhc4eRbN0koLdkOWL53nG";
//
//    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
//    // We will define some constants and conversions here
//    private static final float mmPerInch        = 25.4f;
//    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
//
//    // Constant for Stone Target
//    private static final float stoneZ = 2.00f * mmPerInch;
//
//    // Constants for the center support targets
//    private static final float bridgeZ = 6.42f * mmPerInch;
//    private static final float bridgeY = 23 * mmPerInch;
//    private static final float bridgeX = 5.18f * mmPerInch;
//    private static final float bridgeRotY = 59;                                 // Units are degrees
//    private static final float bridgeRotZ = 180;
//
//    // Constants for perimeter targets
//    private static final float halfField = 72 * mmPerInch;
//    private static final float quadField  = 36 * mmPerInch;
//
//    // Class Members
//    private OpenGLMatrix lastLocation = null;
//    private VuforiaLocalizer vuforia = null;
//    private boolean targetVisible = false;
//    private float phoneXRotate    = 0;
//    private float phoneYRotate    = 0;
//    private float phoneZRotate    = 0;

    ElapsedTime mRuntime = new ElapsedTime();

    //     int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////;
//    public CameraDevice cam;//camera!
//    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    @Override
    public void init() {

        rightFront = hardwareMap.dcMotor.get("right_front");
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftBack = hardwareMap.dcMotor.get("left_back");


        //Hello!  comment the line below if the 'error with imu' thing shows up again
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");


//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection   = CAMERA_CHOICE;
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//

//        colorSensor = hardwareMap.colorSensor.get("colorSensor");
//
        clasp = hardwareMap.servo.get("clasp");
//





        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        motors.add(pulley);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);




        driveToFoundation = new driveState(31 , .8, motors, "forward");
        distanceDriveToFoundation = new distanceState2(motors, distanceSensor, 10 ); //drives to the foundation
        foundationClasp = new OnlyClaspState(clasp, 2,  1); //grabs foundation
        driveToBuildingZone = new driveState(-45 , .8, motors, "backward");
        releaseFoundation = new OnlyClaspState(clasp, 2,  0); //ungrabs foundation
        strafeIntoBuildingZone = new driveState(60, 1, motors, "right");
        pushFoundation = new driveState(25 , .8, motors, "forward");

        // Set all motors to zero power
//        rightFront.setPower(0);
//        leftFront.setPower(0);
//        rightBack.setPower(0);
//        leftBack.setPower(0);

        driveToFoundation.setNextState(foundationClasp);

        foundationClasp.setNextState(driveToBuildingZone);

        driveToBuildingZone.setNextState(strafeIntoBuildingZone);
        strafeIntoBuildingZone.setNextState(releaseFoundation);
        releaseFoundation.setNextState(pushFoundation);
        pushFoundation.setNextState(parkUnderBridge);
    }


    @Override
    public void start(){
        machine = new StateMachine(driveToFoundation);


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
