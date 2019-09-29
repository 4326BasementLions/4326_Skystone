package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.Servo;

public class HugState implements State{
    State NextState;

    DcMotor leftFront;                                    //////////////////////
    DcMotor rightFront;                                  ///////////////////////   :)
    DcMotor leftBack;                                     /////////////////////
    DcMotor rightBack;

    DcMotor pulley;

    Servo leftHand;

    Servo rightHand;

    ElapsedTime mRuntime = new ElapsedTime();

    int time;





    //does the action at start!



    @Override
    public void start() {

        mRuntime.reset();

    }

    @Override
    public State update() {
        //leftFront.setPower();

        if(mRuntime.seconds()<=3) {
            leftHand.setPosition(-1);
            rightHand.setPosition(1);
        }
        if(mRuntime.seconds()<5&&mRuntime.seconds()>3) {
            pulley.setPower(1);
        }




        return NextState;
    }
}
