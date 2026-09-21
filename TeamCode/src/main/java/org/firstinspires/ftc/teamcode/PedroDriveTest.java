package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Pedro Drive Test", group = "Pedro Test")
public class PedroDriveTest extends LinearOpMode {

    private Drivetrain drivetrain;
    private PinpointLocalizer localizer;

    @Override
    public void runOpMode() {

        // Create Pedro drivetrain
        drivetrain = Constants.createDrivetrain(hardwareMap);
        localizer = Constants.createLocalizer(hardwareMap);
        telemetry.addLine("==============================");
        telemetry.addLine("       PEDRO DRIVE TEST");
        telemetry.addLine("==============================");
        telemetry.addLine("");
        telemetry.addLine("Drivetrain initialized");
        telemetry.addLine("");
        telemetry.addLine("Press START");
        telemetry.update();
        localizer.setPose(Pose.zero());

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            // Driver controls
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // Slow mode
            double speed =  0.2;

            forward *= speed;
            strafe *= speed;
            turn *= speed;

            // Pedro drivetrain
            drivetrain.drive(
                    new DrivePowers(
                            forward,
                            strafe,
                            turn
                    ),
                    true
            );

            telemetry.clear();

            telemetry.addLine("==============================");
            telemetry.addLine("       PEDRO DRIVE TEST");
            telemetry.addLine("==============================");

            telemetry.addData("Status", "DRIVETRAIN WORKING");

            telemetry.addLine("");

            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);

            telemetry.addLine("");
            telemetry.addLine("Left Stick Y : Forward");
            telemetry.addLine("Left Stick X : Strafe");
            telemetry.addLine("Right Stick X: Turn");
            telemetry.addLine("Right Bumper : Slow Mode");

            localizer.update();

            // Get pose
            Pose pose = localizer.pose();

            // Convert heading to degrees
            double headingDegrees =
                    Math.toDegrees(pose.heading())+90;

            if(headingDegrees>=360){
                headingDegrees=Math.toDegrees(pose.heading())-270;
            }else {
               headingDegrees= Math.toDegrees(pose.heading())+90;
            }
            telemetry.clear();

            telemetry.addLine("==============================");
            telemetry.addLine("    PEDRO PATHING TEST");
            telemetry.addLine("==============================");

            telemetry.addData(
                    "Pedro Pathing",
                    "WORKING"
            );

            telemetry.addLine("");

            telemetry.addData(
                    "X",
                    "%.2f in",
                    pose.y()
            );

            telemetry.addData(
                    "Y",
                    "%.2f in",
                    pose.x()
            );

            telemetry.addData(
                    "Heading",
                    "%.2f deg",
                    headingDegrees
            );

            telemetry.addLine("");
            telemetry.addLine("------------------------------");
            telemetry.addLine("Movement Test");
            telemetry.addLine("------------------------------");

            telemetry.addLine("Forward  -> X should increase");
            telemetry.addLine("Backward -> X should decrease");
            telemetry.addLine("Left     -> Y should increase");
            telemetry.addLine("Right    -> Y should decrease");

            telemetry.addLine("");
            telemetry.addLine("------------------------------");
            telemetry.addLine("Rotation Test");
            telemetry.addLine("------------------------------");

            telemetry.addLine("CCW -> Heading should increase");
            telemetry.addLine("CW  -> Heading should decrease");

            telemetry.update();
        }

        // Stop motors
        drivetrain.drive(
                new DrivePowers(0, 0, 0),
                true
        );
    }
}