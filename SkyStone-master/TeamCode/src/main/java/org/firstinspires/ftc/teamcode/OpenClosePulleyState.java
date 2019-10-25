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

public class OpenClosePulleyState implements StateMachine.State {
    State NextState;
    DcMotor pulley;
    Servo leftHand;
    Servo rightHand;
    String set;

    private double Time;
    ElapsedTime mRuntime = new ElapsedTime();

    private double Power;

    public OpenClosePulleyState(ArrayList<DcMotor> motor, Servo right, Servo left, String setting){
     //   Time = time;
       // Power = power;
        pulley = motor.get(4);
        leftHand = left;
        rightHand = right;
        set = setting;

    }

    @Override
    public void start() {
        mRuntime.reset();
        if(set.equals("close")){
            leftHand.setPosition(0);
            rightHand.setPosition(1);
        }
        if(set.equals("open")){
            leftHand.setPosition(1);
            rightHand.setPosition(0);
        }


    }
    public void setNextState(StateMachine.State state) {
        NextState = state;

    }
    @Override
    public StateMachine.State update() {
        return NextState;
    }

}
