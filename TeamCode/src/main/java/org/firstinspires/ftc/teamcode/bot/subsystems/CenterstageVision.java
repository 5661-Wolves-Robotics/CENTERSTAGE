package org.firstinspires.ftc.teamcode.bot.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;
import org.firstinspires.ftc.teamcode.util.CameraStream;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CenterstageVision extends SubsystemBase {

    private final WebcamName m_frontCam, m_rearCam;
    private final CenterStagePipeline pipeline;

    private final VisionPortal vision;
    private final AprilTagProcessor aprilTagProcessor;
    private final CameraStream cameraStream;

    private CenterStagePipeline.PropPosition detectedPos = null;

    private final Vector2d m_frontCamPos = new Vector2d(8, 0);
    private final Vector2d m_rearCamPos = new Vector2d(-8, 0);

    public CenterstageVision(
            HardwareMap hardwareMap,
            String frontCam,
            String rearCam
    ){
        m_frontCam = hardwareMap.get(WebcamName.class, frontCam);
        m_rearCam = hardwareMap.get(WebcamName.class, rearCam);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        cameraStream = new CameraStream();

        pipeline = new CenterStagePipeline();

        CameraName switchableCam = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(m_frontCam, m_rearCam);

        vision = new VisionPortal.Builder()
                .setCamera(switchableCam)
                .addProcessor(cameraStream)
                .addProcessor(aprilTagProcessor)
                .addProcessor(pipeline)
                .setCameraResolution(new Size(640, 480))
                .build();

        vision.setActiveCamera(m_frontCam);

        vision.setProcessorEnabled(pipeline, false);
        vision.setProcessorEnabled(aprilTagProcessor, false);

        FtcDashboard.getInstance().startCameraStream(cameraStream, 0);
    }

    public void enablePipelineProcessor(boolean enabled){
        vision.setProcessorEnabled(pipeline, enabled);
    }

    public void enableAprilTagProcessor(boolean enabled){
        vision.setProcessorEnabled(aprilTagProcessor, enabled);
    }

    public void storePropPosition(CenterStagePipeline.PropPosition propPos){
        detectedPos = propPos;
    }

    //1 - RED
    //2 - BLUE
    public void extractColorChannel(int col){
        pipeline.extractChannel(col);
    }

    public CenterStagePipeline.PropPosition getPropPosition(){
        return detectedPos;
    }

    public CenterStagePipeline.PropPosition getPipelinePosition(){
        return pipeline.getPosition();
    }

    public List<AprilTagDetection> getAprilTags(){
        return aprilTagProcessor.getDetections();
    }

    public void cleanUp(){
        vision.close();
    }

    public Vector2d getActiveCamPos(){
        return vision.getActiveCamera() == m_frontCam ? m_frontCamPos : m_rearCamPos;
    }

    public double getActiveCamRot(){
        return vision.getActiveCamera() == m_frontCam ? 0 : -Math.toRadians(180);
    }

}
