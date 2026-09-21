package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Pedro 24in - 30 Percent", group = "Pedro Test")
public class Pedro24InchTest extends LinearOpMode {

    private Follower follower;

    // Maximum requested drive power
    private static final double MAX_POWER = 0.30;

    @Override
    public void runOpMode() {

        follower = Constants.create(hardwareMap);

        // Starting position
        follower.setPose(Pose.zero());

        // 24 inch straight path
        Path path = line(
                Pose.zero(),
                new Pose(24 , 0, 0)
        ).constant(0);

        telemetry.addLine("==============================");
        telemetry.addLine(" PEDRO 24 INCH / 30% TEST");
        telemetry.addLine("==============================");
        telemetry.addLine("");
        telemetry.addData("Maximum Power", "%.0f%%", MAX_POWER * 100);
        telemetry.addData("Target X", "24.00 in");
        telemetry.addData("Target Y", "0.00 in");
        telemetry.addData("Target Heading", "0.00 deg");
        telemetry.addLine("");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        follower.setPose(Pose.zero());
        follower.update();

        follower.follow(path);

        while (opModeIsActive()) {

            follower.update();

            Pose pose = follower.pose();

            telemetry.clear();

            telemetry.addLine("==============================");
            telemetry.addLine(" PEDRO 24 INCH / 30% TEST");
            telemetry.addLine("==============================");

            telemetry.addData(
                    "Status",
                    follower.atParametricEnd()
                            ? "PATH COMPLETE"
                            : "FOLLOWING"
            );

            telemetry.addData(
                    "Limit",
                    "%.0f%%",
                    MAX_POWER * 100
            );

            telemetry.addLine("");

            telemetry.addData(
                    "X",
                    "%.2f in",
                    pose.x()
            );

            telemetry.addData(
                    "Y",
                    "%.2f in",
                    pose.y()
            );

            telemetry.addData(
                    "Heading",
                    "%.2f deg",
                    Math.toDegrees(pose.heading())
            );

            telemetry.addLine("");

            telemetry.addData(
                    "Target X",
                    "24.00 in"
            );

            telemetry.addData(
                    "Error X",
                    "%.2f in",
                    24.0 - pose.x()
            );

            telemetry.update();

            if (follower.atParametricEnd()) {
                sleep(1000);
                break;
            }
        }
    }
}