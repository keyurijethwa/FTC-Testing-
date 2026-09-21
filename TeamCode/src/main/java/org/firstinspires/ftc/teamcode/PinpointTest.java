package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pedro Pinpoint Test", group = "Pedro Test")
public class PinpointTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        PinpointConfig config = new PinpointConfig(c -> {

            // Hardware Map name
            c.name.set("pinpoint");

            // GoBILDA 4-Bar odometry pods
            c.podType.set(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            );

            // Your current offsets
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);

            // X / Forward pod
            c.xPodDirection.set(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            // Y / Strafe pod
            // REVERSED because your Y direction was backwards
            c.yPodDirection.set(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );

            // Units
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        });

        // Create Pedro Pinpoint localizer
        PinpointLocalizer localizer =
                new PinpointLocalizer(hardwareMap, config);

        // Start at 0,0,0
        localizer.setPose(Pose.zero());

        telemetry.addLine("==============================");
        telemetry.addLine("   PEDRO PATHING PINPOINT");
        telemetry.addLine("==============================");
        telemetry.addLine("");
        telemetry.addLine("X  : FORWARD");
        telemetry.addLine("Y  : REVERSED");
        telemetry.addLine("Heading : Pinpoint IMU");
        telemetry.addLine("");
        telemetry.addLine("Pedro Pathing initialized");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Reset starting position
        localizer.setPose(Pose.zero());

        while (opModeIsActive()) {

            // Update localization
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
    }
}