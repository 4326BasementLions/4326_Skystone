package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;

public class OnlyClaspState implements State {
    DcMotor leftFront;                                    //////////////////////
    DcMotor rightFront;                                  ///////////////////////   :)
    DcMotor leftBack;                                     /////////////////////
    DcMotor rightBack;

    Servo arm;
    State NextState;
    ElapsedTime mRuntime = new ElapsedTime();
    Double time;
    double Power;
    String Movement;
    double pos;
    // private StateMachine.State NextState;
    public OnlyClaspState(ArrayList<DcMotor> motor, Servo clasp, double sec, double position){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        arm = clasp;
        time = sec;
        mRuntime.reset();

        pos = position;
    }
    @Override
    public void start() {
        mRuntime.reset();

    }

    @Override
    public State update() {

//        if(arm.getPosition() == 0)
//            arm.setPosition(1);
//        else
        if(mRuntime.seconds()<=time) {
            arm.setPosition(pos);
            return this;
        }
        //(1000);
        return NextState;



    }

    public void setNextState(State state) {
        NextState = state;
    }
}
