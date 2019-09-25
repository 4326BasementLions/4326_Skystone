package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary

import java.security.Policy;
import java.util.ArrayList;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class distanceState implements StateMachine.State{
    DistanceSensor sensorDistance;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    int Distance;
    int Power;
    State NextState;

    public distanceState(int power, int distance, ArrayList<DcMotor> motor){
        Distance = distance;
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        Power = power;

    }
    public void setNextState(State state) {
        NextState  = state;

    }
    public void start(){

    }

    public State update(){
        while(sensorDistance.getDistance(DistanceUnit.INCH)>Distance){
            leftFront.setPower(Power);
            rightFront.setPower(Power);
            leftBack.setPower(Power);
            rightBack.setPower(Power);
            return this;
        }
        if(sensorDistance.getDistance(DistanceUnit.INCH)<=Distance){
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            // return NextState;
        }
        return NextState;
    }

}