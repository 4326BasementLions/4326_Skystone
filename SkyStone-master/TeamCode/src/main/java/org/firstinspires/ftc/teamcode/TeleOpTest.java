package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="New Chassis Test 1.0", group="Iterative Opmode")
public class TeleOpTest extends OpMode
{
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor lift;
    private Servo grab;
    private CRServo twist;

    private float grabPos = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftIntake  = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        lift = hardwareMap.get(DcMotor.class, "lift");
        grab = hardwareMap.get(Servo.class, "grab");
        twist = hardwareMap.get(CRServo.class, "twist");
        grab.setPosition(1);
    }

    public void loop() {

        double intakePower;
        if (gamepad1.right_trigger > 0) {
            intakePower = gamepad1.right_trigger;
        } else if (gamepad1.left_trigger >0) {
            intakePower = -gamepad1.left_trigger;
        } else {
            intakePower = 0;
        }
        rightIntake.setPower(intakePower);
        leftIntake.setPower(-intakePower);

        lift.setPower(gamepad1.right_stick_y);

        if (gamepad1.left_stick_y > 0 && grabPos <= 1)
            grabPos += 0.01;
        else if (gamepad1.left_stick_y < 0 && grabPos >= -1)
            grabPos -= 0.01;
        grab.setPosition(grabPos);

        twist.setPower(gamepad1.left_stick_x);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        lift.setPower(0);
    }

}