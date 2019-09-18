package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ClaspState implements StateMachine.State {
    DcMotor leftFront;                                    //////////////////////
    DcMotor rightFront;                                  ///////////////////////   :)
    DcMotor leftBack;                                     /////////////////////
    DcMotor rightBack;
    private StateMachine.State NextState;
    @Override
    public void start() {

    }

    @Override
    public StateMachine.State update() {
      return NextState;

    }
}
