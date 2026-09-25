package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_d {

    private Servo s;
    private Servo s2;

    private Servo clr;

    public servo_d(HardwareMap hardwareMap){
        s=hardwareMap.get(Servo.class,"RS");
        s2=hardwareMap.get(Servo.class,"RS2");
        clr=hardwareMap.get(Servo.class,"CLED");

    }
    public void setPB(){
        s.setPosition(0);

    }
    public void setPA(){
        s.setPosition(1);

    }
    public void setHalf(){
        s.setPosition(0.5);
    }
    public void setPY(){
        s2.setPosition(0.7);
    }
    public void setPX(){
        s2.setPosition(0.5);
    }

    public double getP(){
        return s.getPosition();
    }
}
