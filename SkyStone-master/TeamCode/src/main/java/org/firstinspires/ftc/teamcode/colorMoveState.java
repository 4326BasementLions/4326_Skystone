//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.StateMachine.State;

public class colorMoveState implements State {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    String cval;
    State NextState;
    ColorSensor cs1;
    String dir;
    int cntr = 0;
    int passNum;
    boolean justSaw = false;
    String turn = "null";
    boolean isMoved = false;
    String detectedColor = "none";

    public colorMoveState(ArrayList<DcMotor> motor, ColorSensor colorSensor, String color, int passes, String direction) {
        this.leftFront = (DcMotor)motor.get(0);
        this.rightFront = (DcMotor)motor.get(1);
        this.leftBack = (DcMotor)motor.get(2);
        this.rightBack = (DcMotor)motor.get(3);
        this.cs1 = colorSensor;
        this.cval = color;
        this.passNum = passes;
        this.dir = direction;
    }

    public void setNextState(State state) {
        this.NextState = state;
    }

    public void start() {
    }

    public String getTurn() {
        return this.turn;
    }

    public State update() {
        move(dir, .2);
        this.checkColor();
        if (this.detectedColor.equals(this.cval) && this.cntr == this.passNum) {
            stop(leftFront,leftBack,rightBack,rightFront);
            //move("forward",0);
            return NextState;
        } else if (this.detectedColor.equals(this.cval) && this.cntr != this.passNum && !this.justSaw) {
            this.justSaw = true;
            return this;
        } else if (this.detectedColor.equals(this.cval) && this.cntr != this.passNum && this.justSaw) {
            return this;
        } else if (!this.detectedColor.equals(this.cval) && this.cntr != this.passNum && this.justSaw) {
            ++this.cntr;
            this.justSaw = false;
            return this;
        } else {
            return this.detectedColor != this.cval ? this : this;
        }
    }

    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        motorlf.setPower(0.0D);
        motorrf.setPower(0.0D);
        motorlb.setPower(0.0D);
        motorrb.setPower(0.0D);
    }

    public void checkColor() {
        if (this.cs1.red() > this.cs1.blue() && this.cs1.red() > this.cs1.green()) {
            this.detectedColor = "red";
        } else if (this.cs1.blue() > this.cs1.red() && this.cs1.blue() > this.cs1.green()) {
            this.detectedColor = "blue";
        } else {
            this.detectedColor = "none";
        }

    }

    public String getColor() {
        return this.detectedColor;
    }

    public boolean hasJustSeen() {
        return this.justSaw;
    }

    public int getCntr() {
        return this.cntr;
    }

    public void move(String direction, double speed) {
        byte var5 = -1;
        switch(direction.hashCode()) {
            case -677145915:
                if (direction.equals("forward")) {
                    var5 = 0;
                }
                break;
            case 3188:
                if (direction.equals("cw")) {
                    var5 = 5;
                }
                break;
            case 98327:
                if (direction.equals("ccw")) {
                    var5 = 4;
                }
                break;
            case 3317767:
                if (direction.equals("left")) {
                    var5 = 3;
                }
                break;
            case 108511772:
                if (direction.equals("right")) {
                    var5 = 2;
                }
                break;
            case 2121976803:
                if (direction.equals("backward")) {
                    var5 = 1;
                }
        }

        switch(var5) {
            case 0:
                this.leftFront.setPower(-speed);
                this.rightFront.setPower(-speed);
                this.leftBack.setPower(-speed);
                this.rightBack.setPower(-speed);
                break;
            case 1:
                this.leftFront.setPower(speed);
                this.rightFront.setPower(speed);
                this.leftBack.setPower(speed);
                this.rightBack.setPower(speed);
                break;
            case 2:
                this.leftFront.setPower(-speed);
                this.rightFront.setPower(speed);
                this.leftBack.setPower(-speed);
                this.rightBack.setPower(speed);
                break;
            case 3:
                this.leftFront.setPower(speed);
                this.rightFront.setPower(-speed);
                this.leftBack.setPower(speed);
                this.rightBack.setPower(-speed);
                break;
            case 4:
                this.leftFront.setPower(speed);
                this.rightFront.setPower(-speed);
                this.leftBack.setPower(speed);
                this.rightBack.setPower(-speed);
                break;
            case 5:
                this.leftFront.setPower(-speed);
                this.rightFront.setPower(speed);
                this.leftBack.setPower(-speed);
                this.rightBack.setPower(speed);
        }

    }
}
