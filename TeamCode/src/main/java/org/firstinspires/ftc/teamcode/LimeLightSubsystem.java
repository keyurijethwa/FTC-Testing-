package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimeLightSubsystem {
    private Limelight3A limelight;
    int tagId = -1;

    LLResult result ;
    public LimeLightSubsystem(HardwareMap hardwareMap){

        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);

        limelight.start();
        result = limelight.getLatestResult();
    }

    public void getLimelightId(){
        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            for (LLResultTypes.FiducialResult fiducial : fiducials) {

                int tagId = fiducial.getFiducialId();

                telemetry.addData("AprilTag ID", tagId);
            }
        }

    }

    public boolean getDetected(){
        boolean aprilTagDetected =
                result != null && result.isValid();
        return aprilTagDetected;
    }
}
