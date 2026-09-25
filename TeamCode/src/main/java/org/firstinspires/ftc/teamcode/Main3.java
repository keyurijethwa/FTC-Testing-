package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "DriveTeleOp", group = "FTC")
public class Main3 extends LinearOpMode {

    private Intake_Balls intake;
    private ShooterSubsystem1 s;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // ==========================================
        // HARDWARE INITIALIZATION
        // ==========================================

        intake = new Intake_Balls(hardwareMap);
        s = new ShooterSubsystem1(hardwareMap);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        // AprilTag pipeline
        limelight.pipelineSwitch(0);

        limelight.start();


        // ==========================================
        // TIMER VARIABLES
        // ==========================================

        boolean lastRightBumper = false;

        boolean timerRunning = false;
        boolean timerFinished = false;

        double startTime = 0;
        double finalTime = 0;


        // ==========================================
        // WAIT FOR START
        // ==========================================

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Limelight", "Starting...");
        telemetry.update();

        waitForStart();


        // ==========================================
        // MAIN LOOP
        // ==========================================

        while (opModeIsActive()) {

            // ==========================================
            // LIMELIGHT RESULT
            // ==========================================

            LLResult result = limelight.getLatestResult();

            List<LLResultTypes.FiducialResult> fiducials = null;

            if (result != null) {
                fiducials = result.getFiducialResults();
            }


            // ==========================================
            // APRILTAG DETECTION
            // ==========================================

            boolean aprilTagDetected =
                    fiducials != null && !fiducials.isEmpty();


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

                s.shootFast();

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


            // ==========================================
            // LIMELIGHT TELEMETRY
            // ==========================================

            telemetry.addData(
                    "Limelight Result",
                    result != null
            );

            telemetry.addData(
                    "AprilTag Detected",
                    aprilTagDetected
            );


            // ==========================================
            // PRINT ALL APRILTAG IDs
            // ==========================================

            if (aprilTagDetected) {

                telemetry.addData(
                        "Number of Tags",
                        fiducials.size()
                );

                for (int i = 0; i < fiducials.size(); i++) {

                    LLResultTypes.FiducialResult fiducial =
                            fiducials.get(i);

                    int detectedId =
                            fiducial.getFiducialId();

                    telemetry.addData(
                            "Tag " + (i + 1),
                            "ID = " + detectedId
                    );
                }

            } else {

                telemetry.addData(
                        "AprilTags",
                        "NONE"
                );
            }


            // ==========================================
            // UPDATE TELEMETRY ONCE
            // ==========================================

            telemetry.update();
        }


        // ==========================================
        // STOP
        // ==========================================

        s.stop();
        intake.stop1();
        limelight.stop();
    }
}