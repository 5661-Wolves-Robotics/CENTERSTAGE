package org.firstinspires.ftc.teamcode.bot.commands;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

import java.util.function.DoubleSupplier;

public class PlacerDrive extends CommandBase {

    private final PIDFController headingController = new PIDFController(Mecanum.HEADING_PID);

    private final double SPEED = 0.3;
    private final FieldConstants.Side SIDE;

    private final Mecanum m_drive;
    private final DoubleSupplier m_horizontal;
    private final DoubleSupplier m_vertical;
    private final DoubleSupplier m_rot;

    private double targetRot = Math.PI;

    public PlacerDrive(MecanumDriveBase driveBase, DoubleSupplier horizontal, DoubleSupplier vertical, DoubleSupplier rot, FieldConstants.Side side){
        m_drive = driveBase.getDrive();
        m_horizontal = horizontal;
        m_vertical = vertical;
        m_rot = rot;
        SIDE = side;

        headingController.setInputBounds(-Math.PI, Math.PI);
        headingController.setTargetPosition(targetRot);

        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        int scale = SIDE == FieldConstants.Side.RED ? 1 : -1;

        targetRot += -m_rot.getAsDouble() * 0.02;
        headingController.setTargetPosition(targetRot);

        m_drive.setFieldCentricWeightedDrivePower(new Pose2d(
                m_horizontal.getAsDouble() * SPEED * scale,
                m_vertical.getAsDouble() * SPEED * scale,
                (headingController.update(m_drive.getPoseEstimate().getHeading()) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH
        ));
    }
}
