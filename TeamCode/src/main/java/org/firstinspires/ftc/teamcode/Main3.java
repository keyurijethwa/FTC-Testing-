package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "DriveTeleOp", group = "FTC")
public class Main3 extends LinearOpMode {

    private Intake_Balls intake;
    private ShooterSubsystem1 s;

    @Override
    public void runOpMode() {

        intake = new Intake_Balls(hardwareMap);
        s = new ShooterSubsystem1(hardwareMap);

        // ==============================
        // TIMER VARIABLES
        // ==============================

        boolean lastRightBumper = false;

        boolean timerRunning = false;
        boolean timerFinished = false;

        double startTime = 0;
        double finalTime = 0;

        waitForStart();

        while (opModeIsActive()) {

            // ==========================================
            // INTAKE CONTROL
            // ==========================================

            if (gamepad1.left_bumper) {

                intake.in(0.7);

            } else if (gamepad1.left_trigger > 0.1) {

                intake.out(0.7);

            } else {

                intake.stop1();
            }


            // ==========================================
            // SHOOTER BUTTON
            // ==========================================

            boolean rightBumper =
                    gamepad1.right_bumper;


            // ==========================================
            // NEW RIGHT BUMPER PRESS
            // ==========================================

            if (rightBumper && !lastRightBumper) {

                // Start shooter
                s.shootFast();

                // Start timer
                startTime = getRuntime();

                timerRunning = true;
                timerFinished = false;
                finalTime = 0;
            }


            // ==========================================
            // SHOOTER RUNNING
            // ==========================================

            if (rightBumper) {

                s.shootFast();

                double velocity =
                        s.getAverageVelocity();


                // ======================================
                // STOP TIMER AT 1260
                // ======================================

                if (timerRunning && velocity >= 1260) {

                    finalTime =
                            getRuntime() - startTime;

                    timerRunning = false;
                    timerFinished = true;
                }

            } else {

                // Bumper released
                s.stop();

                timerRunning = false;
            }


            // Save bumper state
            lastRightBumper = rightBumper;


            // ==========================================
            // SHOOTER TELEMETRY
            // ==========================================

            telemetry.addData(
                    "Left Shooter",
                    "%.0f",
                    s.getLeftVelocity()
            );

            telemetry.addData(
                    "Right Shooter",
                    "%.0f",
                    s.getRightVelocity()
            );

            telemetry.addData(
                    "Average Velocity",
                    "%.0f",
                    s.getAverageVelocity()
            );

            telemetry.addData(
                    "Target Velocity",
                    "1280"
            );

            telemetry.addData(
                    "Timer Target",
                    "1260"
            );


            // ==========================================
            // TIMER TELEMETRY
            // ==========================================

            if (timerRunning) {

                double currentTime =
                        getRuntime() - startTime;

                telemetry.addData(
                        "SPIN-UP TIME",
                        "%.3f sec",
                        currentTime
                );

                telemetry.addData(
                        "STATUS",
                        "SPINNING..."
                );

            } else if (timerFinished) {

                telemetry.addData(
                        "SPIN-UP TIME",
                        "%.3f sec",
                        finalTime
                );

                telemetry.addData(
                        "STATUS",
                        "1260 REACHED"
                );

            } else {

                telemetry.addData(
                        "SPIN-UP TIME",
                        "Press RIGHT BUMPER"
                );

                telemetry.addData(
                        "STATUS",
                        "READY"
                );
            }


            telemetry.update();
        }

        s.stop();
        intake.stop1();
    }
}