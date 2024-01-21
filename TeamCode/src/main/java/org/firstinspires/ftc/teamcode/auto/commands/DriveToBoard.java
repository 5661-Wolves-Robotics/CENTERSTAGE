package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;

public class DriveToBoard extends CommandBase {

    private final Mecanum m_drive;
    private final CenterstageVision m_cv;

    private final FieldConstants.Side m_side;
    private final FieldConstants.Stage m_stage;

    public DriveToBoard(
            MecanumDriveBase driveBase,
            CenterstageVision cv,
            FieldConstants.Side side,
            FieldConstants.Stage stage
    ){
        m_drive = driveBase.getDrive();
        m_cv = cv;

        m_side = side;
        m_stage = stage;

        addRequirements(driveBase, cv);
    }

    @Override
    public void initialize() {
        CenterStagePipeline.PropPosition propPos = m_cv.getPropPosition();

        boolean redSide = m_side == FieldConstants.Side.RED;
        double yScale = redSide ? 1 : -1;

        //For BLUE side swap left and right prop positions
        if(!redSide){
            if(propPos == CenterStagePipeline.PropPosition.LEFT) propPos = CenterStagePipeline.PropPosition.RIGHT;
            else if(propPos == CenterStagePipeline.PropPosition.RIGHT) propPos = CenterStagePipeline.PropPosition.LEFT;
        }

        if(m_stage == FieldConstants.Stage.BACK) {
            switch (propPos) {
                case LEFT:
                    m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate(), true)
                            .back(3)
                            .splineTo(new Vector2d(51, -29 * yScale), 0)
                            .build());
                    break;
                case CENTER:
                    m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate(), true)
                            .back(3)
                            .splineTo(new Vector2d(51, -36 * yScale), 0)
                            .build());
                    break;
                case RIGHT:
                    m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate(), true)
                            .splineToLinearHeading(new Pose2d(51, -44 * yScale, Math.toRadians(180)), Math.toRadians(270))
                            .build());
                    break;
            }
        } else {
            switch (propPos) {
                case LEFT:
                    m_drive.followTrajectorySequence(m_drive.trajectorySequenceBuilder(m_drive.getPoseEstimate())
                            .setReversed(true)
                            .back(7)
                            .turn(-Math.toRadians(90))
                            .lineTo(new Vector2d(30, -12 * yScale))
                            .splineToLinearHeading(new Pose2d(51, -30 * yScale, Math.toRadians(180)), Math.toRadians(0))
                            .build());
                    break;
                case CENTER:
                    m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate(), true)
                            .back(5)
                            .splineTo(new Vector2d(-36, -12 * yScale), 0)
                            .lineTo(new Vector2d(30, -12 * yScale))
                            .splineToLinearHeading(new Pose2d(51, -36 * yScale, Math.toRadians(180)), Math.toRadians(0))
                            .build());
                    break;
                case RIGHT:
                    m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate(), true)
                            .back(5)
                            .splineTo(new Vector2d(-24, -12 * yScale), 0)
                            .lineTo(new Vector2d(30, -12 * yScale))
                            .splineToLinearHeading(new Pose2d(51, -42 * yScale, Math.toRadians(180)), Math.toRadians(0))
                            .build());
                    break;
            }
        }
        m_drive.followTrajectory(m_drive.trajectoryBuilder(m_drive.getPoseEstimate())
                .back(6)
                .build());
    }

    @Override
    public boolean isFinished() {
        return m_drive.done();
    }
}
