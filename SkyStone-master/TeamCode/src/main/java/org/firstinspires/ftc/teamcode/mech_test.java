/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="NJ Test", group="Iterative Opmode")
public class        mech_test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    private Servo left ;
    private Servo right = null;
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;


    DcMotor pulley;

    Servo leftHand;

    CRServo rightHand;
    Servo clasp;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    double x = .5;
    double z = .5;
    double n = .5;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
      //  lift  = hardwareMap.get(DcMotor.class, "lift");
    //    left = hardwareMap.get(Servo.class, "left");
       // right = hardwareMap.get(Servo.class, "right");




        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
      //  telemetry.addData("leftpos", left.getPosition());

//        rightFront = hardwareMap.dcMotor.get("right front");
//        leftFront = hardwareMap.dcMotor.get("left front");
//        rightBack = hardwareMap.dcMotor.get("right back");
//        leftBack = hardwareMap.dcMotor.get("left back");
        rightHand = hardwareMap.crservo.get("right");
        leftHand = hardwareMap.servo.get("left");
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.setDirection(DcMotor.Direction.REVERSE);
        pulley = hardwareMap.dcMotor.get("pulley");
       // clasp = hardwareMap.servo.get("clasp");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        leftHand.setPosition(.5);
       // rightHand.(.5);

        runtime.reset();
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        pulley.setPower(-gamepad2.left_stick_y/4);



        if(gamepad2.right_trigger>0){
//        Unfortunately, using .getPos within the .setPos method lead to an infinite loop, as .getPos would constantly update
            x = x + .025;
            //z = z - .025;
        }
        if(gamepad2.left_trigger>0){
            x = x - .025;
           // z = z + .025;
//

        }
        if(gamepad1.right_trigger>0){
            //n = n + .025;
            z = z - .025;
        }
        if(gamepad1.left_trigger>0){
           // n = n - .025;
            z = z + .025;
        }

//        if(x>1){
//            x=1;
//
//        }
//        if(x<0){
//            x=0;
//        }
        if(z>1){
            z=1;
        }
        if(z<0){
            z=0;
        }
//        if(n>1){
//            n=1;
//        }
//        if(n<0){
//            n=0;
//        }



//
//if(gamepad2.right_stick_y>0){
//    x+=.01;
//    z-=.01;
//}
//if(gamepad2.right_stick_y<0){
//            x-=.01;
//            z+=.01;
//}
//else if(gamepad2.x){
//    x=.5;j
//    z=.5;
//
//}
//        leftHand.setPosition(gamepad1.left_stick_y);
//
//        rightHand.setPosition(gamepad1.right_stick_y);

//        float drive = gamepad1.right_stick_y;
//        float strafe = gamepad1.right_stick_x;
//        float turn = gamepad1.left_stick_x;
//
//        float fl = drive - strafe + turn;
//        float fr = drive + strafe - turn;
//        float bl = drive + strafe + turn;
//        float br = drive - strafe - turn;
//
//        leftFront.setPower(fl);
//        rightFront.setPower(fr);
//        leftBack.setPower(bl);
//        rightBack.setPower(br);


//


leftHand.setPosition(z);
rightHand.setPower(-gamepad1.left_stick_y);
//clasp.setPosition(n);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
