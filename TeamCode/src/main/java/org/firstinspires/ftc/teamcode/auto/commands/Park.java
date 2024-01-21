package org.firstinspires.ftc.teamcode.auto.commands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutonomousMain;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

public class Park extends CommandBase {

    private final Mecanum m_drive;
    private final FieldConstants.EnumSupplier<FieldConstants.Side> m_sideSupplier;
    private final FieldConstants.EnumSupplier<FieldConstants.Stage> m_stageSupplier;

    public Park(
            MecanumDriveBase driveBase,
            FieldConstants.EnumSupplier<FieldConstants.Side> sideSupplier,
            FieldConstants.EnumSupplier<FieldConstants.Stage> stageSupplier
    ){
        m_sideSupplier = sideSupplier;
        m_stageSupplier = stageSupplier;

        m_drive = driveBase.getDrive();

        addRequirements(driveBase);
    }

    @Override
    public void initialize() {
        double xScale = m_sideSupplier.getVal() == FieldConstants.Side.RED ? 1 : -1;

        if(m_stageSupplier.getVal() == FieldConstants.Stage.BACK){
            m_drive.followTrajectoryAsync(m_drive.trajectoryBuilder(m_drive.getPoseEstimate())
                    .splineToConstantHeading(new Vector2d(48, -60 * xScale), Math.toRadians(180))
                    .build()
            );
        } else {
            m_drive.followTrajectoryAsync(m_drive.trajectoryBuilder(m_drive.getPoseEstimate())
                    .splineToConstantHeading(new Vector2d(48, -12 * xScale), Math.toRadians(180))
                    .build()
            );
        }
    }

    @Override
    public void execute() {
        m_drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        AutonomousMain.storePosition(m_drive.getPoseEstimate());
    }

    @Override
    public boolean isFinished() {
        return m_drive.done();
    }
}
