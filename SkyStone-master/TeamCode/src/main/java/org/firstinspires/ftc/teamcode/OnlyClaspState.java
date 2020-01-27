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
    int time;
    double Power;
    String Movement;
    double pos;

    // private StateMachine.State NextState;
    public OnlyClaspState( Servo clasp, int sec, double position){

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
