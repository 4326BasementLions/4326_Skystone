package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import java.util.Locale;
import com.qualcomm.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;

public class ClaspState implements StateMachine.State {
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
    public ClaspState(ArrayList<DcMotor> motor, Servo clasp, double sec, String move, double power, double position){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        arm = clasp;
        time = sec;
        mRuntime.reset();
        Power = power;
        Movement = move;
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
        while (mRuntime.seconds() < time) {

//            arm.setPosition(pos);
            if (Movement == "forward") {
                //arm.setPosition(pos);
                    //for some reason == worked
                leftFront.setPower(Power);
                rightFront.setPower(Power);
                leftBack.setPower(Power);
                rightBack.setPower(Power);
            }
            if (Movement == "backward") {
                //on = true;
                //arm.setPosition(pos);

                leftFront.setPower(-Power);
                rightFront.setPower(-Power);
                leftBack.setPower(-Power);
                rightBack.setPower(-Power);
            }
            if (Movement == "turnLeft") {
                leftFront.setPower(-Power);
                rightFront.setPower(Power);
                leftBack.setPower(-Power);
                rightBack.setPower(Power);

            }
            if (Movement == "turnRight") {
                leftFront.setPower(Power);
                rightFront.setPower(-Power);
                leftBack.setPower(Power);
                rightBack.setPower(-Power);

            }
         //   arm.setPosition(pos);
            return this;
        }
        if(time<=mRuntime.seconds()){
            //on = false;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            // return NextState;
        }


        return NextState;



    }

    public void setNextState(State state) {
        NextState = state;
    }
}
