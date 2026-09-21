package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Ordometry")
public class Ordometry extends OpMode {

    GoBildaPinpointDriver pinpoint;
    @Override
    public void init() {
        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        configurePinpoint();

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0));
    }

    @Override
    public void loop() {
        telemetry.addLine("Push your robot around to see it track");
        telemetry.addLine("Press A to reset the position");
        if(gamepad1.a){
            // You could use readings from April Tags here to give a new known position to the pinpoint
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
    }

    public void configurePinpoint(){
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

}
