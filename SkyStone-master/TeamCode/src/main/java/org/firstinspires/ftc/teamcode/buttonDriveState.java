package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DigitalChannel;
public class buttonDriveState implements State {

    DcMotor frontRight;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    double power;
    DigitalChannel touchSensor;

    State NextState;

    public buttonDriveState(ArrayList<DcMotor> mtr, double pwr, DigitalChannel touch){
        frontRight = mtr.get(0);
        frontLeft = mtr.get(1);
        backRight = mtr.get(2);
        backLeft = mtr.get(3);
        power = pwr;
        touchSensor = touch;

    }



    @Override
    public void start() {


    }

    public void setNextState(State nextState){
        NextState = nextState;
    }

    @Override
    public State update() {
        if(touchSensor.getState()==true){
            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);
            return this;

        } else {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
            return NextState;
        }




    }
}
