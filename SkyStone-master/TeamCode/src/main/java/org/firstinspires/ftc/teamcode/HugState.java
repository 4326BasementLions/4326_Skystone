package org.firstinspires.ftc.teamcode;

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

public class HugState implements State{
    int newleftBackTarget;
    int newrightBackTarget;
    int  newleftFrontTarget;
    int  newrightFrontTarget;
    int pulleyTarget;
    double distance;

    State NextState;

    DcMotor leftFront;                                    //////////////////////
    DcMotor rightFront;                                  ///////////////////////   :)
    DcMotor leftBack;                                     /////////////////////
    DcMotor rightBack;

    DcMotor pulley;

    Servo leftHand;

    Servo rightHand;

    ElapsedTime mRuntime = new ElapsedTime();

    int time;

    double pow;

    boolean open;


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static double driveSpeed = 0.6;
    static final double TURN_SPEED = 0.5;



    //does the action at start!
    public HugState(ArrayList<DcMotor> motor, int speed /*let's make this encoder based asap*/, Servo left, Servo right, double Pow){//, boolean tf){
        driveSpeed = speed;
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        pulley = motor.get(4);

        leftHand = left;
        rightHand = right;

//        time = sec;

        //open = tf;

        pow = Pow;

    }



    @Override
    public void start() {
        //Reset the encoders back to zero for the next movement
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        rightBack.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.REVERSE);

        //Bring them back to using encoders
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Setting their target to their current encoder value (should be zero) to the amount of inches times the counts per inches

        newleftBackTarget = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newrightBackTarget = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newleftFrontTarget = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newrightFrontTarget = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        pulleyTarget = pulley.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        mRuntime.reset();

    }

    @Override
    public State update() {

        if(pulleyTarget > pulley.getCurrentPosition()) {
            pulley.setPower(driveSpeed);
            leftHand.setPosition(-1);
            rightHand.setPosition(1);
            return this;
        }

        //if(mRuntime.seconds()<time){
//            if(open){
//                leftHand.setPosition(1);
//                rightHand.setPosition(-1);
//            }
//            else{
//                leftHand.setPosition(-1);
//                rightHand.setPosition(1);
//            }
//            pulley.setPower(pow);

            //return this; //>_<
       // }
        //leftFront.setPower();

//        if(mRuntime.seconds()<=3) {
//            leftHand.setPosition(-1);
//            rightHand.setPosition(1);
//        }
//        if(mRuntime.seconds()<5&&mRuntime.seconds()>3) {
//            pulley.setPower(1);
//        }




        return NextState;
    }
}
