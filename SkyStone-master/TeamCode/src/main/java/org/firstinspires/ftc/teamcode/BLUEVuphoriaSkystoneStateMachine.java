package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.ColorSenseStopState;
import org.firstinspires.ftc.teamcode.DoubleClaspState;
import org.firstinspires.ftc.teamcode.GyroTurnCCWByPID;
import org.firstinspires.ftc.teamcode.GyroTurnCWByPID;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import org.firstinspires.ftc.teamcode.buttonDriveState;
import org.firstinspires.ftc.teamcode.driveState;
import org.firstinspires.ftc.teamcode.motorStateSwitch;

import java.util.ArrayList;




@Autonomous(name="BLUE - TrueCameraBlockMachine",group="Iterative Opmode")

public class BLUEVuphoriaSkystoneStateMachine extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor VuforiaTest;

    DigitalChannel touchSensor;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true  ;

    float xpos;
    float ypos;
    float zpos;


    String currState;
    boolean enabled = false;

    ElapsedTime mRuntime = new ElapsedTime();

    BNO055IMU imu;

    private Servo foundationLeft;
    private Servo foundationRight;

    ColorSensor colorSensor;




    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
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

    // Class Members]
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

    private OpenGLMatrix lastLocation = null;
    //    private VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
    private VuforiaLocalizer vuforia;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;



    // Next, translate the camera lens to where it is on the robot.
    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
    final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
    final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line


    //  Instantiate the Vuforia engine
    VuforiaTrackables targetsSkyStone;
    VuforiaTrackable stoneTarget;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();



    private StateMachine machine;
    private StateMachine machine0;

    motorStateSwitch testee;
    motorStateSwitch test2;
    buttonDriveState touchTest;
    driveState firstStep;
    GyroTurnCCWByPID firstTurn;
    driveState goforth;
    driveState goback;

    // Second machine states

    GyroTurnCWByPID faceBlock;
    DoubleClaspState grabbbbbbbbbbb;
    driveState goBusterst;
    GyroTurnCCWByPID facebridge;
    ColorSenseStopState park;



    @Override
    public void init() {


        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        stoneTarget = targetsSkyStone.get(0);
        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsSkyStone);
        stoneTarget.setName("Stone Target");
        //    tester = hardwareMap.servo.get("tester");
        VuforiaTest = hardwareMap.dcMotor.get("motor");
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftBack = hardwareMap.dcMotor.get("left_back");

        foundationLeft = hardwareMap.servo.get("foundation_left");
        foundationRight = hardwareMap.servo.get("foundation_right");

        colorSensor = hardwareMap.colorSensor.get("color_sensor");


        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        touchSensor = hardwareMap.get(DigitalChannel.class, "sensor_digital");


        testee = new motorStateSwitch(VuforiaTest, -.5, false, 30);

        testee.setNextState(null);



        //touchTest = new buttonDriveState(VuforiaTest, -.5, touchSensor);
        test2 = new motorStateSwitch(VuforiaTest, .5, false, 30);

        touchTest.setNextState(test2);


        firstStep = new driveState(30, .5, motors, "forward");
        firstTurn = new GyroTurnCCWByPID(90,.5, motors, imu);
        goforth = new driveState(30,.5,motors,"forward");
        goback = new driveState(30, .5, motors, "backward");

        firstStep.setNextState(firstTurn);
        firstTurn.setNextState(goforth);
        goforth.setNextState(goback);

        //second machine
        faceBlock = new GyroTurnCWByPID(90, .5, motors, imu);
        grabbbbbbbbbbb = new DoubleClaspState(foundationLeft, foundationRight, 2, 0);
        goBusterst = new driveState(10, .5, motors, "backward");
        facebridge = new GyroTurnCCWByPID(90, .5, motors, imu);
        park = new ColorSenseStopState(motors, colorSensor, "blue", .5, "forward");




        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));



        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }


    }
    @Override
    public void start() {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        machine = new StateMachine(faceBlock);

        machine0 = new StateMachine(firstStep);

        machine0.update();

        targetsSkyStone.activate();


        mRuntime.reset();

    }



    public void loop(){

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {



            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            xpos = translation.get(0) / mmPerInch;
            ypos = translation.get(1) / mmPerInch;
            zpos = translation.get(2) /mmPerInch;
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    xpos, ypos, zpos);
            //these are redundant, but ah well
            telemetry.addData("x", xpos);   ;
            telemetry.addData("y", ypos);   ;
            telemetry.addData("z", zpos);/*o*/;



            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);


            while (machine0.currentState() == touchTest) {

                // send the info back to driver station using telemetry function.
                // if the digital channel returns true it's HIGH and the button is unpressed.
                if (touchSensor.getState() == true) {
                    telemetry.addData("Digital Touch", "Is Not Pressed");
                } else {
                    telemetry.addData("Digital Touch", "Is Pressed");
                }

                telemetry.update();
            }

        }
        if (!enabled){
            machine0.update();
        }

        if(xpos<-10&&!enabled){

            machine.update();
            enabled = true;
            mRuntime.reset();

        }
        if(enabled){

            machine.update();
        }
        telemetry.addData("state", machine.currentState());
        telemetry.update();



    }

}

