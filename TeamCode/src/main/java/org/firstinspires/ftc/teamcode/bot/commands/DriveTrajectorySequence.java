package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.util.TrajectorySequenceSupplier;

public class DriveTrajectorySequence extends CommandBase {

    private final Mecanum m_drive;
    private final TrajectorySequenceSupplier m_trajectory;

    public DriveTrajectorySequence(MecanumDriveBase driveBase, TrajectorySequenceSupplier trajectory){
        m_drive = driveBase.getDrive();
        m_trajectory = trajectory;

        addRequirements(driveBase);
    }

    @Override
    public void initialize() {
        m_drive.followTrajectorySequenceAsync(m_trajectory.getTrajectory(m_drive));
    }

    @Override
    public boolean isFinished() {
        return m_drive.done();
    }
}
