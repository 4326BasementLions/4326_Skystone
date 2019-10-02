package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="SimpleDrive", group="Iterative Opmode")
public class SimpleDrive extends OpMode {
    DcMotor left;
    DcMotor right;
    public void init(){
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        left.setDirection(DcMotor.Direction.REVERSE);

    }

    public void loop(){
        left.setPower(gamepad1.left_stick_y);
        right.setPower(gamepad1.right_stick_y);


    }

    @Override
    public void start() {
        //start();
    }
}
