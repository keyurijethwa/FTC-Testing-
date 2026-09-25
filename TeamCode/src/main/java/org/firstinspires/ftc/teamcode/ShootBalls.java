package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShootBalls {

    private DcMotorEx SL;
    private DcMotorEx SR;

    private double targetVelocity=1200;
    private PIDFCoefficients check;

    public ShootBalls(HardwareMap hardwareMap){
        SL=hardwareMap.get(DcMotorEx.class,"SL");
        SR=hardwareMap.get(DcMotorEx.class,"SR");

        SL.setDirection(DcMotorSimple.Direction.FORWARD);
        SR.setDirection(DcMotorSimple.Direction.FORWARD);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf =
                new PIDFCoefficients(100, 0, 5, 15.5);

        SL.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        SR.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );
//        SL.setVelocityPIDFCoefficients(170, 0, 5, 15.5);
//        SR.setVelocityPIDFCoefficients(170, 0, 5, 15.5);
        check = SL.getPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER
        );
    }

    public void stop(){
        SL.setPower(0);
        SR.setPower(0);
    }

    public void forward(double power){

//        if (currentVelocity >= targetVelocity) {
//
//            SL.setVelocity(targetVelocity);
//            SR.setVelocity(targetVelocity);
//
//        } else {
//
//            // Don't suddenly stop.
//            // Reduce velocity target slightly.
//            SL.setVelocity(targetVelocity - 100);
//            SR.setVelocity(targetVelocity - 100);
//        }
//        if (currentVelocity >= targetVelocity) {
//
//            SL.setPower(0);
//            SR.setPower(0);
//
//        } else {
//
//            SL.setVelocity(targetVelocity);
//            SR.setVelocity(targetVelocity);
//        }

//        if (currentVelocity <targetVelocity) {
//            // Hold velocity strictly at or slightly below targetVelocity
//            SL.setVelocity(targetVelocity);
//            SR.setVelocity(targetVelocity);
//        } else {
//            // Smoothly approach target without overshooting
//            double error = targetVelocity - currentVelocity;
//
//            // If we are close (within 100 ticks/sec), step target down to prevent overshoot
//            if (error < 100) {
//                SL.setVelocity(currentVelocity + (error * 0.5));
//                SR.setVelocity(currentVelocity + (error * 0.5));
//            } else {
//                SL.setVelocity(targetVelocity);
//                SR.setVelocity(targetVelocity);
//            }
//        }
//        if(currentVelocity>=targetVelocity){
//            SL.setVelocity(targetVelocity * 0.95); // Briefly back off 5% to settle speed
//            SR.setVelocity(targetVelocity * 0.95);
//        }else{
//            SL.setVelocity(targetVelocity);
//            SR.setVelocity(targetVelocity);
//        }
//
        SL.setPower(power);
        SR.setPower(power);

    }
    public void reverse(double power){
        SL.setPower(-power);
        SR.setPower(-power);
    }

    public double getLeftVelocity() {
        return Math.abs(SL.getVelocity());
    }

    public double getRightVelocity() {
        return Math.abs(SR.getVelocity());
    }
    public double getVelocity(){
        double total=(SL.getVelocity()+ SR.getVelocity())/2;
        return Math.abs(total);
    }

    public double getLPvalue(){
        return SL.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).p;
    }
    public PIDFCoefficients getLIvalue(){
        return check;
    }
    public double getLDvalue(){
        return SL.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).d;
    }
    public double getLFvalue(){
        return SL.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).f;
    }
    public double getRPvalue(){
        return SR.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).p;
    }
    public double getRIvalue(){
        return SR.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).i;
    }
    public double getRDvalue(){
        return SR.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).d;
    }
    public double getRFvalue(){
        return SR.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER).f;
    }
}
