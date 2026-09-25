package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Main2")
public class Main2 extends LinearOpMode {

    private Intake_Balls i;
    @Override
    public void runOpMode() throws InterruptedException {
        i= new Intake_Balls(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.left_bumper){
                i.in(0.7);
            } else{
                i.stop1();
            }
            if (gamepad1.left_trigger>0.2) {
                i.out(0.7);
            } else {
                i.stop1();
            }
            telemetry.addData("Get a left bumper",gamepad1.left_bumper);
            telemetry.addData("Get a left trigger",gamepad1.left_trigger);
            telemetry.update();
        }
    }
}
