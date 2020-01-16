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

public class ModernTeleOp extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo foundationLeft;
    Servo foundationRight;

    DcMotor pulley;
    Servo twist;
    Servo grab;

    DcMotor intakeLeft;
    DcMotor intakeRight;

    double x;
    double y;

    boolean grip = false;


    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightFront = hardwareMap.dcMotor.get("right_front");

        foundationLeft = hardwareMap.servo.get("foundation_left");
        foundationRight = hardwareMap.servo.get("foundation_right");

        pulley = hardwareMap.dcMotor.get("pulley");
        twist = hardwareMap.servo.get("twist");
        grab = hardwareMap.servo.get("grab");

        intakeLeft = hardwareMap.dcMotor.get("intake_left");
        intakeRight = hardwareMap.dcMotor.get("intake_right");


        x=0;

        y=1;

    }

    @Override
    public void start() {


    }

    @Override
    public void loop() {

    //GAMEPAD 1
        //Drive train
            float drive = gamepad1.right_stick_y;
            float strafe = gamepad1.right_stick_x;
            float turn = gamepad1.left_stick_x;

            float fl = drive - strafe + turn;
            float fr = drive + strafe - turn;
            float bl = drive + strafe + turn;
            float br = drive - strafe - turn;

            leftFront.setPower(fl);
            rightFront.setPower(fr);
            leftBack.setPower(bl);
            rightBack.setPower(br);

        //Foundation Clasps (triggers)

            foundationLeft.setPosition(x);
            foundationRight.setPosition(x);

            if(gamepad1.left_trigger>0){
                x+=.25;
            }
            if(gamepad1.right_trigger>0){
                x-=.25;
            }

    //GAMEPAD 2
        //Intake
            if(gamepad2.left_trigger>0){
                intakeRight.setPower(1);
                intakeLeft.setPower(1);
            }
            if(gamepad2.right_trigger>0){
                intakeRight.setPower(-1);
                intakeLeft.setPower(-1);
            }

        //Pulley
            pulley.setPower(gamepad2.right_stick_y/4);

        //Twist+Grab
            if(gamepad2.left_stick_x>0){
                y+=.25;
            }
            if(gamepad2.left_stick_x>0){
                y-=.25;
            }

            twist.setPosition(y);

            if(gamepad2.right_bumper&&grip){
                grab.setPosition(0);  //let go of block
            }
            if(gamepad2.right_bumper&&!grip){
                grab.setPosition(1);  //hold on to block
            }


    }
}
