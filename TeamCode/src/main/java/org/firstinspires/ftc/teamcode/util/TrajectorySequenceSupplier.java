package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@FunctionalInterface
public interface TrajectorySequenceSupplier {
    TrajectorySequence getTrajectory(Mecanum drive);
}
