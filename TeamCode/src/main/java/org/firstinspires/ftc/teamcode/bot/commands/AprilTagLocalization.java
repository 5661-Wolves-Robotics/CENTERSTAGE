package org.firstinspires.ftc.teamcode.bot.commands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagLocalization extends CommandBase {

    private final MecanumDriveBase m_driveBase;
    private final CenterstageVision m_cv;

    public AprilTagLocalization(CenterstageVision cv, MecanumDriveBase driveBase){
        m_driveBase = driveBase;
        m_cv = cv;

        m_cv.enablePipelineProcessor(false);
        m_cv.enableAprilTagProcessor(true);

        addRequirements(cv);
    }

    @Override
    public void execute() {
        List<AprilTagDetection> detections = m_cv.getAprilTags();
        int num = detections.size();

        Vector2d camPos = m_cv.getActiveCamPos();
        double camRot = m_cv.getActiveCamRot();

        if(num > 0) {
            Vector2d pos = new Vector2d(0, 0);
            double bearing = 0;

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    Vector2d camPosEstimate = new Vector2d(detection.metadata.fieldPosition.get(0), detection.metadata.fieldPosition.get(1))
                            .minus(new Vector2d(detection.ftcPose.x, detection.ftcPose.y));
                    camPos = camPos.rotated(detection.ftcPose.bearing);
                    pos = pos.plus(camPosEstimate.minus(camPos));
                    bearing += detection.metadata.fieldOrientation.z - (Math.toRadians(180) + camRot) + detection.ftcPose.bearing;
                }
            }

            Pose2d estimate = new Pose2d(pos.div(num), bearing / num);

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStroke("red")
                    .setStrokeWidth(2)
                    .setRotation(estimate.getHeading())
                    .strokeRect(estimate.getX(), estimate.getY(), 18, 18);
        }
    }
}
