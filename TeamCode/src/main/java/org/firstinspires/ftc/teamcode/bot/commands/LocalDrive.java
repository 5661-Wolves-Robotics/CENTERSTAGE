package org.firstinspires.ftc.teamcode.bot.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.util.collision.shapes.Box;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LocalDrive extends CommandBase {

    private final Mecanum m_drive;
    private final DoubleSupplier m_x;
    private final DoubleSupplier m_y;
    private final DoubleSupplier m_rot;
    private final BooleanSupplier m_colliding;

    private final ElapsedTime time;

    public LocalDrive(MecanumDriveBase driveBase, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier colliding){
        m_drive = driveBase.getDrive();
        m_x = x;
        m_y = y;
        m_rot = rot;
        m_colliding = colliding;

        time = new ElapsedTime();

        addRequirements(driveBase);
    }

    double lastTime = 0;
    double deltaTime = 0;

    @Override
    public void execute() {
        if(m_colliding.getAsBoolean()) {
            Vector2d driveInput = new Vector2d(m_y.getAsDouble(), -m_x.getAsDouble());
            Pose2d estimate = m_drive.getPoseEstimate();
            Vector2d inputEstimate = driveInput.times(deltaTime).times(DriveConstants.MAX_VEL).rotated(estimate.getHeading());
            Box bounds = new Box(new Pose2d(estimate.vec().plus(inputEstimate), estimate.getHeading()), 18, 18);

            Vector2d collisionVec = FieldConstants.collision.checkBox(bounds);

            if (collisionVec != null) {
                double heading = estimate.getHeading();
                Vector2d input = driveInput.rotated(heading);

                Vector2d collisionNorm = new Vector2d(-Math.abs(collisionVec.getX()), collisionVec.getY());
                m_drive.setWeightedDrivePower(new Pose2d(
                        collisionNorm.div(collisionNorm.norm()).times(input.norm()).plus(input).rotated(-heading),
                        m_rot.getAsDouble()
                ));
            } else {
                m_drive.setWeightedDrivePower(new Pose2d(
                        m_y.getAsDouble(),
                        -m_x.getAsDouble(),
                        -m_rot.getAsDouble()
                ));
            }
        } else {
            m_drive.setWeightedDrivePower(new Pose2d(
                    m_y.getAsDouble(),
                    -m_x.getAsDouble(),
                    -m_rot.getAsDouble()
            ));

            double newTime = time.seconds();
            deltaTime = newTime - lastTime;
            lastTime = newTime;
        }
    }
}
