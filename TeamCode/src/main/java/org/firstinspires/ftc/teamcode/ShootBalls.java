package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShootBalls {

    private DcMotor SL;
    private DcMotor SR;
    public ShootBalls(HardwareMap hardwareMap){
        SL=hardwareMap.get(DcMotor.class,"SL");
        SR=hardwareMap.get(DcMotor.class,"SR");

        SL.setDirection(DcMotorSimple.Direction.FORWARD);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stop(){
        SL.setPower(0);
        SR.setPower(0);
    }

    public void forward(double power){
        SL.setPower(power);
        SR.setPower(power);
    }
    public void reverse(double power){
        SL.setPower(-power);
        SR.setPower(-power);
    }
}
