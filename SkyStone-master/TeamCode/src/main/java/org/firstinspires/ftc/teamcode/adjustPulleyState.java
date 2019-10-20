package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;

public class adjustPulleyState implements StateMachine.State {

    DcMotor pulley;
    Servo leftHand;
    Servo rightHand;
    boolean on = false;

    private double Power;
    private String Movement;

    private State NextState;

    private double Time;
    ElapsedTime mRuntime = new ElapsedTime();


    public adjustPulleyState(double time, double power, ArrayList<DcMotor> motor, Servo right, Servo left) {
        Time = time;
        pulley = motor.get(4);
        leftHand = left;
        rightHand = right;
        Power = power;
        mRuntime.reset();

    }

    public void setNextState(State state) {
        NextState = state;

    }

    @Override
    public void start() {
        //  this.update();
        mRuntime.reset();
        pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftHand.setPosition(0);
        rightHand.setPosition(1);


    }

    @Override
    public StateMachine.State update() {


        while (mRuntime.seconds() < Time) {
            pulley.setPower(Power);
            return this;
        }


        //on = false;
        pulley.setPower(0);

        return NextState;
    }
    public boolean getOn(){
        return on;
    }
}




