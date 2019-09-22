package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.StateMachine.State;

public class ColorSenseStopState implements State {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    String cval;
    State NextState;
    ColorSensor cs1;
    String dir;
    double pow;
    int red;
    public ColorSenseStopState(ArrayList<DcMotor> motor, ColorSensor colorSensor, String color, double power, String direction){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        cs1 = colorSensor;
        cval = color;
        pow = power;
        dir = direction;
        //red = cs1.red();
    }

    public void setNextState(State state) {
        NextState = state;
    }

    public State update(){

        leftBack.setPower(pow);
        leftFront.setPower(pow);
        rightBack.setPower(pow);
        rightFront.setPower(pow);

        if(cs1.red()> 1000 && cs1.red()>cs1.blue() && cs1.red()>cs1.green()){
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
            return NextState;
        }

        return this;
    }

    public int getColor(){

        return cs1.red();

    }

    @Override
    public void start() {

    }
}
