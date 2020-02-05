package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Modern Tele op Test :)", group="Iterative Opmode")
public class ModernTeleOpTest extends OpMode {
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

    double grabPos = 0.5;
    int grabState = 0;
    int twistState = 0;
    int grabCooldown = 0;
    int twistCooldown = 0;

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
        //ADD OTHER SERVO FOR BLOCK DRAGGING
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
        float drive = gamepad1.right_stick_x;
        float strafe = -gamepad1.left_stick_x;
        float turn = gamepad1.left_stick_y;

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
            intakeLeft.setPower(-1);
        } else if(gamepad2.right_trigger>0) {
            intakeRight.setPower(-1);
            intakeLeft.setPower(1);
        } else {
            intakeRight.setPower(0);
            intakeLeft.setPower(0);
        }
////
////        //Pulley
        pulley.setPower(gamepad2.right_stick_y/2);
////
////        //Grab
        if (grabCooldown == 0 && gamepad2.a) {
            grabState ^= 1;  //how does this shift grabState?
            grabPos = grabState;
            grab.setPosition(grabPos);
            //changed grabCooldown from 400 to 200s
            grabCooldown = 200;
        } else if (grabCooldown > 0){
            grabCooldown--;
        }

        if(twistCooldown == 0 && gamepad2.x){
            twistState ^=1;
            twist.setPosition(twistState);
            twistCooldown = 400;
        }else if (twistCooldown > 0) {
            twistCooldown--;
        }

        if (gamepad2.b) {
            grab.setPosition(0.5);
        }

        telemetry.addData("grab pos", grabPos);
        telemetry.addData("grab cd", grabCooldown);
        telemetry.addData("grab state", grabState);
        telemetry.update();
    }

    public void flipBlock() {
        grab.setPosition(0);

        grab.setPosition(.5);
    }
}
