package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake_Balls {
    private final DcMotorEx intake;

    public Intake_Balls(HardwareMap hardwareMap){
        intake=hardwareMap.get(DcMotorEx.class,"IB");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void in(double power){
        intake.setPower(power);
    }
    public void out(double power){
        intake.setPower(-power);
    }
    public void stop1(){
        intake.setPower(0);
    }
    public double invelo(){

        return Math.abs(intake.getVelocity());
    }

}
