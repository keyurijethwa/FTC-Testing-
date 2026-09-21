package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "LimeLight")
public class Limelight_test extends LinearOpMode {

    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight=hardwareMap.get(Limelight3A.class,"limelight");

        limelight.pipelineSwitch(0);

        limelight.start();
        waitForStart();
        while (opModeIsActive()){
            LLResult result=limelight.getLatestResult();
            if(result!=null){
                if(result.isValid()){
                    Pose3D botpose=result.getBotpose();
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (!fiducials.isEmpty()) {
                        LLResultTypes.FiducialResult tag = fiducials.get(0);

                        // Access target 3D pose relative to camera
                        Pose3D targetPose = tag.getTargetPoseCameraSpace();
                        if (targetPose != null) {
                            double x = targetPose.getPosition().x;
                            double y = targetPose.getPosition().y;
                            double z = targetPose.getPosition().z;

                            // Perform formula calculation: OP = sqrt(x^2 + y^2 + z^2)
                            double opMeters = Math.sqrt(x * x + y * y + z * z);

                            // Convert results to centimeters
                            double opCm = opMeters * 100.0;
                            double forwardCm = z * 100.0;

                            telemetry.addData("Direct Distance OP (cm)", opCm);
                            telemetry.addData("Forward Distance Z (cm)", forwardCm);
                        }
                    }else {
                        telemetry.addData("Limelight", "No Target Detected");
                    }

                    telemetry.addData("tx",result.getTx());
                    telemetry.addData("ty",result.getTy());
                    telemetry.addData("Botpose",botpose.toString());
                    telemetry.update();
                }
            }
        }

    }
}
