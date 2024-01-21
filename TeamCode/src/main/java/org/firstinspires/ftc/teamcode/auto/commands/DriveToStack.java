package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.bot.commands.DriveTrajectorySequence;
import org.firstinspires.ftc.teamcode.bot.commands.LowerIntake;
import org.firstinspires.ftc.teamcode.bot.commands.PositionIntake;
import org.firstinspires.ftc.teamcode.bot.commands.RaiseIntake;
import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

public class DriveToStack extends SequentialCommandGroup {

    public DriveToStack(MecanumDriveBase driveBase, Intake intake, FieldConstants.Side side){

        double yScale = side == FieldConstants.Side.RED ? 1 : -1;

        addCommands(
                new DriveTrajectorySequence(driveBase, drive -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(30, -11 * yScale), Math.toRadians(180))
                        .lineTo(new Vector2d(-57, -11 * yScale))
                        .build()
                ),
                new PositionIntake(intake, 0.35),
                new WaitCommand(800),
                new LowerIntake(intake),
                new DriveTrajectorySequence(driveBase, drive -> drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setReversed(true)
                        .lineTo(new Vector2d(30, -11 * yScale))
                        .splineToLinearHeading(new Pose2d(50, -36 * yScale, Math.toRadians(180)), 0)
                        .back(2, Mecanum.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), Mecanum.getAccelerationConstraint(10))
                        .build()
                ),
                new RaiseIntake(intake)
        );

        addRequirements(driveBase, intake);
    }

}
