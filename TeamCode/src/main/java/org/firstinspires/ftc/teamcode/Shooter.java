package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private final DcMotor shooter;
    public  Shooter(HardwareMap hardwareMap){
        shooter=hardwareMap.get(DcMotor.class,"");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void shoot(double power){
        shooter.setPower(power);
    }
}
