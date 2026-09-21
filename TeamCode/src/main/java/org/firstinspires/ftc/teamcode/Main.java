package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.test_robot;

@TeleOp(name = "Main")
public class Main extends LinearOpMode {
    private test_robot tr;
    private Intake_Balls ib;
    private servo_d s;
    double drive_power;
    double intake_power;
    private Servo clr;

    GoBildaPinpointDriver pinpoint;


    @Override
    public void runOpMode() throws InterruptedException {
        tr=new test_robot(hardwareMap);
        ib=new Intake_Balls(hardwareMap);
        s=new servo_d(hardwareMap);

        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        clr=hardwareMap.get(Servo.class,"CLED");

        drive_power=0.8;
        intake_power=0.2;

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            tr.driveRobot(
                    -gamepad1.left_stick_y * drive_power,
                    gamepad1.left_stick_x * drive_power,
                    -gamepad1.right_stick_x * drive_power
            );
            if(gamepad1.left_bumper){
                // You could use readings from April Tags here to give a new known position to the pinpoint
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
            if(gamepad1.left_trigger>0.2){

                tr.turn90(90,0,Math.toDegrees(pinpoint.getHeading(AngleUnit.DEGREES)));

            }
            pinpoint.update();

            telemetry.addData(
                    "X (in)",
                    "%.2f",
                    pinpoint.getPosX(DistanceUnit.INCH));

            telemetry.addData(
                    "Y (in)",
                    "%.2f",
                    pinpoint.getPosY(DistanceUnit.INCH));

            telemetry.addData(
                    "Heading (deg)",
                    "%.2f",
                    pinpoint.getHeading(AngleUnit.DEGREES));

            telemetry.addData(
                    "Frequency",
                    "%.0f Hz",
                    pinpoint.getFrequency());

            telemetry.update();
            if(gamepad1.right_bumper ){
                ib.in(intake_power);
                if(ib.invelo()>150) {
                    clr.setPosition(0.5);
                }

            } else if  (gamepad1.right_trigger>0.2){
                ib.out(intake_power);
                if(ib.invelo()>150) {
                    clr.setPosition(0.722);
                }
            }else {
                ib.stop1();
                if (ib.invelo()==0){
                    clr.setPosition(0);
                }
            }

            if(gamepad1.b){
                s.setPB();
                if(s.getP()==0){
                    clr.setPosition(0.333);
                }else {
                    clr.setPosition(0);
                }

            }
            if(gamepad1.a){
                s.setPA();
                if(s.getP()==1){
                    clr.setPosition(0.555);
                }else{
                    clr.setPosition(0);
                }
            }
            if(gamepad1.y){
                s.setPY();
            }
            if (gamepad1.x){
                s.setPX();
            }




        }

    }
}
