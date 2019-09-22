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
    // private StateMachine.State NextState;
    public ClaspState(ArrayList<DcMotor> motor, Servo clasp, double sec){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        arm = clasp;
        time = sec;
    }
    @Override
    public void start() {

    }

    @Override
    public State update() {
        while (mRuntime.seconds() < time) {
            arm.setPosition(0);
        }
//        if(arm.getPosition() == 0)
//            arm.setPosition(1);
//        else


      return NextState;

    }

    public void setNextState(State state) {
        NextState = state;
    }
}
