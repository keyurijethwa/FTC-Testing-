package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * LIMELIGHT POSITION LOCK TEST
 *
 * A     = Lock current position
 * B     = Unlock position
 *
 * The robot must see an AprilTag through Limelight
 * before the position can be locked.
 *
 * Pinpoint provides:
 *      X position
 *      Y position
 *      Heading
 *
 * The robot then uses PID-style correction to return
 * to the locked position if it is pushed by hand.
 */
@TeleOp(name = "Limelight Position Lock", group = "TEST")
public class LimelightPositionLock extends LinearOpMode {

    // ============================================================
    // HARDWARE
    // ============================================================

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;


    // ============================================================
    // POSITION LOCK
    // ============================================================

    private boolean positionLocked = false;

    // Locked robot position
    private double lockedX = 0.0;
    private double lockedY = 0.0;
    private double lockedHeading = 0.0;


    // ============================================================
    // PID CONSTANTS
    // ============================================================

    /**
     * X/Y proportional gain.
     *
     * Increase if robot returns too slowly.
     * Decrease if robot becomes aggressive.
     */
    public static double KP_POSITION = 0.035;

    /**
     * Heading proportional gain.
     */
    public static double KP_HEADING = 0.015;


    // ============================================================
    // MAXIMUM CORRECTION POWER
    // ============================================================

    /**
     * Maximum translational correction.
     */
    public static double MAX_TRANSLATION = 0.50;

    /**
     * Maximum rotational correction.
     */
    public static double MAX_ROTATION = 0.35;


    // ============================================================
    // POSITION TOLERANCE
    // ============================================================

    /**
     * Robot is considered at target if X/Y error
     * is smaller than this value.
     */
    public static double POSITION_TOLERANCE = 0.75; // inches

    /**
     * Heading tolerance.
     */
    public static double HEADING_TOLERANCE = 2.0; // degrees


    // ============================================================
    // MAIN
    // ============================================================

    @Override
    public void runOpMode() {

        // --------------------------------------------------------
        // Initialize hardware
        // --------------------------------------------------------

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );


        // --------------------------------------------------------
        // Motor directions
        //
        // Change these if your robot drives incorrectly.
        // --------------------------------------------------------

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        // --------------------------------------------------------
        // Brake mode
        // --------------------------------------------------------

        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );


        // --------------------------------------------------------
        // Initialize Pinpoint
        // --------------------------------------------------------

        pinpoint.update();


        // --------------------------------------------------------
        // Start Limelight
        // --------------------------------------------------------

        limelight.start();


        telemetry.addLine("================================");
        telemetry.addLine(" LIMELIGHT POSITION LOCK");
        telemetry.addLine("================================");
        telemetry.addLine("");
        telemetry.addLine("A = LOCK POSITION");
        telemetry.addLine("B = UNLOCK");
        telemetry.addLine("");
        telemetry.addLine("Waiting for START...");
        telemetry.update();


        waitForStart();


        // ========================================================
        // MAIN LOOP
        // ========================================================

        while (opModeIsActive()) {

            // ----------------------------------------------------
            // Update Pinpoint
            // ----------------------------------------------------

            pinpoint.update();


            // ----------------------------------------------------
            // Read Limelight
            // ----------------------------------------------------

            LLResult result = limelight.getLatestResult();

            boolean aprilTagDetected =
                    result != null && result.isValid();


            // ----------------------------------------------------
            // Current robot position
            // ----------------------------------------------------

            double currentX =
                    pinpoint.getPosX(DistanceUnit.INCH);

            double currentY =
                    pinpoint.getPosY(DistanceUnit.INCH);

            double currentHeading =
                    pinpoint.getHeading(AngleUnit.DEGREES);


            // ----------------------------------------------------
            // A = LOCK POSITION
            //
            // Only lock if Limelight currently detects a target.
            // ----------------------------------------------------

            if (gamepad1.a && !positionLocked) {

                if (aprilTagDetected) {

                    lockedX = currentX;
                    lockedY = currentY;
                    lockedHeading = currentHeading;

                    positionLocked = true;

                }
            }


            // ----------------------------------------------------
            // B = UNLOCK POSITION
            // ----------------------------------------------------

            if (gamepad1.b) {

                positionLocked = false;

                stopRobot();
            }


            // ----------------------------------------------------
            // POSITION LOCK ACTIVE
            // ----------------------------------------------------

            if (positionLocked) {
                if(gamepad1.right_trigger>0.2) {
                    correctToLockedPosition(
                            currentX,
                            currentY,
                            currentHeading
                    );
                }

            } else {

                // ------------------------------------------------
                // Normal manual driving
                // ------------------------------------------------

                manualDrive();
            }


            // ----------------------------------------------------
            // TELEMETRY
            // ----------------------------------------------------

            telemetry.addLine("==============================");

            telemetry.addData(
                    "Limelight",
                    aprilTagDetected ? "APRILTAG DETECTED" : "NO TAG"
            );

            telemetry.addLine("");

            telemetry.addData(
                    "Current X",
                    "%.2f in",
                    currentX
            );

            telemetry.addData(
                    "Current Y",
                    "%.2f in",
                    currentY
            );

            telemetry.addData(
                    "Current Heading",
                    "%.2f°",
                    currentHeading
            );

            telemetry.addLine("");

            telemetry.addData(
                    "Position Lock",
                    positionLocked ? "LOCKED" : "UNLOCKED"
            );

            if (positionLocked) {

                telemetry.addData(
                        "Locked X",
                        "%.2f in",
                        lockedX
                );

                telemetry.addData(
                        "Locked Y",
                        "%.2f in",
                        lockedY
                );

                telemetry.addData(
                        "Locked Heading",
                        "%.2f°",
                        lockedHeading
                );

                telemetry.addLine("");

                telemetry.addData(
                        "X Error",
                        "%.2f in",
                        lockedX - currentX
                );

                telemetry.addData(
                        "Y Error",
                        "%.2f in",
                        lockedY - currentY
                );

                telemetry.addData(
                        "Heading Error",
                        "%.2f°",
                        angleError(
                                lockedHeading,
                                currentHeading
                        )
                );
            }

            telemetry.addLine("");

            telemetry.addData(
                    "A",
                    "LOCK"
            );

            telemetry.addData(
                    "B",
                    "UNLOCK"
            );

            telemetry.update();
        }


        // --------------------------------------------------------
        // Stop everything
        // --------------------------------------------------------

        stopRobot();

        limelight.stop();
    }


    // ============================================================
    // POSITION CORRECTION
    // ============================================================

    /**
     * Calculates the error between current position and
     * locked position and drives the robot toward the target.
     */
    private void correctToLockedPosition(
            double currentX,
            double currentY,
            double currentHeading) {


        // --------------------------------------------------------
        // Calculate position errors
        // --------------------------------------------------------

        double errorX = lockedX - currentX;
        double errorY = lockedY - currentY;

        double errorHeading =
                angleError(
                        lockedHeading,
                        currentHeading
                );


        // --------------------------------------------------------
        // Check if robot is already at target
        // --------------------------------------------------------

        boolean positionOK =
                Math.abs(errorX) < POSITION_TOLERANCE &&
                        Math.abs(errorY) < POSITION_TOLERANCE;

        boolean headingOK =
                Math.abs(errorHeading) < HEADING_TOLERANCE;


        if (positionOK && headingOK) {

            stopRobot();

            return;
        }


        // --------------------------------------------------------
        // Calculate correction
        // --------------------------------------------------------

        double xPower =
                errorX * KP_POSITION;

        double yPower =
                errorY * KP_POSITION;

        double rotationPower =
                errorHeading * KP_HEADING;


        // --------------------------------------------------------
        // Limit correction
        // --------------------------------------------------------

        xPower = clip(
                xPower,
                -MAX_TRANSLATION,
                MAX_TRANSLATION
        );

        yPower = clip(
                yPower,
                -MAX_TRANSLATION,
                MAX_TRANSLATION
        );

        rotationPower = clip(
                rotationPower,
                -MAX_ROTATION,
                MAX_ROTATION
        );


        // --------------------------------------------------------
        // Convert field-relative X/Y correction into
        // robot-relative mecanum movement.
        // --------------------------------------------------------

        double headingRadians =
                Math.toRadians(currentHeading);

        double robotX =
                xPower * Math.cos(headingRadians)
                        + yPower * Math.sin(headingRadians);

        double robotY =
                -xPower * Math.sin(headingRadians)
                        + yPower * Math.cos(headingRadians);


        // --------------------------------------------------------
        // Mecanum calculation
        // --------------------------------------------------------

        double fl =
                robotY + robotX + rotationPower;

        double fr =
                robotY - robotX - rotationPower;

        double bl =
                robotY - robotX + rotationPower;

        double br =
                robotY + robotX - rotationPower;


        // --------------------------------------------------------
        // Normalize motor powers
        // --------------------------------------------------------

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(fl),
                        Math.max(
                                Math.abs(fr),
                                Math.max(
                                        Math.abs(bl),
                                        Math.abs(br)
                                )
                        )
                )
        );


        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;


        // --------------------------------------------------------
        // Apply motor powers
        // --------------------------------------------------------

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }


    // ============================================================
    // MANUAL MECANUM DRIVE
    // ============================================================

    /**
     * Normal driver-controlled mecanum drive.
     */
    private void manualDrive() {

        double y =
                -gamepad1.left_stick_y;

        double x =
                gamepad1.left_stick_x;

        double rx =
                gamepad1.right_stick_x;


        double denominator =
                Math.max(
                        Math.abs(y)
                                + Math.abs(x)
                                + Math.abs(rx),
                        1.0
                );


        double fl =
                (y + x + rx)
                        / denominator;

        double fr =
                (y - x - rx)
                        / denominator;

        double bl =
                (y - x + rx)
                        / denominator;

        double br =
                (y + x - rx)
                        / denominator;


        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }


    // ============================================================
    // ANGLE ERROR
    // ============================================================

    /**
     * Calculates the shortest angular distance between
     * target heading and current heading.
     *
     * Example:
     *
     * Target = 179°
     * Current = -179°
     *
     * Error = -2° instead of 358°.
     */
    private double angleError(
            double target,
            double current) {

        double error =
                target - current;


        while (error > 180) {
            error -= 360;
        }


        while (error < -180) {
            error += 360;
        }


        return error;
    }


    // ============================================================
    // STOP ROBOT
    // ============================================================

    /**
     * Stops all drive motors.
     */
    private void stopRobot() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    // ============================================================
    // CLIP VALUE
    // ============================================================

    /**
     * Limits a value between minimum and maximum.
     */
    private double clip(
            double value,
            double min,
            double max) {

        return Math.max(
                min,
                Math.min(max, value)
        );
    }
}