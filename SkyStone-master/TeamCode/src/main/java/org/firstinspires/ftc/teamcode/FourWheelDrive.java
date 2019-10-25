package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="wheeltest", group="Iterative Opmode")
public class FourWheelDrive extends OpMode {
    DcMotor fl;
    DcMotor bl;
    DcMotor br;
    DcMotor fr;
    public void init(){
        fr = hardwareMap.dcMotor.get("right front");
       fl = hardwareMap.dcMotor.get("left front");
        br = hardwareMap.dcMotor.get("right back");
        bl = hardwareMap.dcMotor.get("left back");
    }

    public void loop(){
        fr.setPower(gamepad1.right_stick_y);
        br.setPower(gamepad2.right_stick_y);
        fl.setPower(gamepad1.left_stick_y);
        bl.setPower(gamepad2.left_stick_y);
    }

    @Override
    public void start() {

    }
}
