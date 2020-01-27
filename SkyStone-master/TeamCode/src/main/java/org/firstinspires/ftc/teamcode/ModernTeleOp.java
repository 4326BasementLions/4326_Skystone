package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Modern Tele op", group="Iterative Opmode")
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

        foundationLeft = hardwareMap.servo.get("foundation");
        //foundationRight = hardwareMap.servo.get("foundation_right");

        pulley = hardwareMap.dcMotor.get("pulley");
        twist = hardwareMap.servo.get("twist");  //not configged yet
        grab = hardwareMap.servo.get("grab"); //not configged yet

        intakeLeft = hardwareMap.dcMotor.get("intake_left");
        intakeRight = hardwareMap.dcMotor.get("intake_right");


        x=0;

        y=1;

    }

    @Override
    public void start() {
//        foundationRight.setPosition(x);
//        foundationLeft.setPosition(x);
//        twist.setPosition(y);
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
////
////        //Foundation Clasps (triggers)
////
            foundationLeft.setPosition(x); //   :(
           // foundationRight.setPosition(x);
//
            if(gamepad1.left_trigger>0&&!(x>=1)){
                x=1;
            }
            if(gamepad1.right_trigger>0&&!(x<=-1)){
                x=-1;
            }

////
////    //GAMEPAD 2
////        //Intake
            if(gamepad2.left_trigger>0){
                intakeRight.setPower(1);
                intakeLeft.setPower(1);
            }
            if(gamepad2.right_trigger>0){
                intakeRight.setPower(-1);
                intakeLeft.setPower(-1);
            }
////
////        //Pulley
            pulley.setPower(gamepad2.right_stick_y/4);
////
////        //Twist+Grab
            if(gamepad2.left_stick_x>0&&!(y>=1)){ //flick right
                y=1;
            } else if(gamepad2.left_stick_x<0&&!(y<=-1)){  //flick left
                y=-1;
            } else {
                y=0;
            }
////
            twist.setPosition(y);

            if(gamepad2.right_bumper&&grip){
                grab.setPosition(0);  //let go of block
                grip = false;
            }
            if(gamepad2.right_bumper&&!grip){
                grab.setPosition(1);  //hold on to block
                grip = true;
            }
            telemetry.addData("number", gamepad1.right_trigger);


    }
}
