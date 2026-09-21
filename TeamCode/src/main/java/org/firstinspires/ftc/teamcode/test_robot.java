package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class test_robot {

    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;

    double headingOffset = 0.0;
    double gettingRawHeading;

public test_robot(HardwareMap hardwareMap){


        fl = hardwareMap.get(DcMotor.class, "FL");
        bl = hardwareMap.get(DcMotor.class, "BL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        br = hardwareMap.get(DcMotor.class, "BR");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

}
   public void driveRobot(double drive,double strafe,double turn){

        double FL = drive + strafe + turn;
        double FR = drive - strafe - turn;
        double BL = drive - strafe + turn;
        double BR = drive + strafe - turn;

        fl.setPower(FL);
        fr.setPower(FR);
        bl.setPower(BL);
        br.setPower(BR);

    }

    public void turn90(double targetHeading,double stableLoops,double rawHeading){
        gettingRawHeading=rawHeading;
         double KP = 0.015;

        // Maximum turning power
        double MAX_TURN_POWER = 0.5;

        // Minimum turning power
        final double MIN_TURN_POWER = 0.08;

        // How close we need to be to the target
        final double HEADING_TOLERANCE = 1.0;

        // Number of loops heading must remain within tolerance
        final int REQUIRED_STABLE_LOOPS = 5;




        double currentHeading = getHeading();

        double error =
                angleWrap(targetHeading - currentHeading);

        // P controller
        double turnPower = error * KP;

        // Limit power
        turnPower = Range.clip(
                turnPower,
                -MAX_TURN_POWER,
                MAX_TURN_POWER
        );

        // Minimum power to overcome friction
        if (Math.abs(error) > HEADING_TOLERANCE) {

            if (Math.abs(turnPower) < MIN_TURN_POWER) {

                turnPower =
                        Math.copySign(
                                MIN_TURN_POWER,
                                turnPower
                        );
            }
        }

        // -------------------------------------------------
        // Rotate robot
        // -------------------------------------------------

        fl.setPower(turnPower);
        bl.setPower(turnPower);

        fr.setPower(-turnPower);
        br.setPower(-turnPower);

        // -------------------------------------------------
        // Check if target reached
        // -------------------------------------------------

        if (Math.abs(error) <= HEADING_TOLERANCE) {

            stableLoops++;

            if (stableLoops >= REQUIRED_STABLE_LOOPS) {

                stopDrive();



                stableLoops = 0;
            }

        } else {

            stableLoops = 0;
        }
    }
    private void stopDrive() {

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    private double getHeading() {

        return angleWrap(
                gettingRawHeading - headingOffset
        );
    }

    private double angleWrap(double angle) {

        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }
    public void turn90LeftEncoder(int targetTicks, double power, LinearOpMode opMode) {
        // 1. Reset encoder positions
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 2. Set target positions (Left wheels go backward, Right wheels go forward)
        fl.setTargetPosition(-targetTicks);
        bl.setTargetPosition(-targetTicks);
        fr.setTargetPosition(targetTicks);
        br.setTargetPosition(targetTicks);

        // 3. Switch motors to RUN_TO_POSITION mode
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 4. Set power to begin movement
        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(power);

        // 5. Wait until motors reach target position or OpMode ends
        while (opMode.opModeIsActive() && (fl.isBusy() || fr.isBusy())) {
            opMode.telemetry.addData("FL Target", targetTicks);
            opMode.telemetry.addData("FL Current", fl.getCurrentPosition());
            opMode.telemetry.update();
        }

        // 6. Stop all motors
        driveRobot(0, 0, 0);

        // 7. Reset back to standard mode for TeleOp driving
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
