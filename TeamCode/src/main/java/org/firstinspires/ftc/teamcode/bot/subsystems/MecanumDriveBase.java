package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;

public class MecanumDriveBase extends SubsystemBase {

    private final Mecanum m_drive;
    private Pose2d startPose;

    public MecanumDriveBase(HardwareMap hardwareMap){
        m_drive = new Mecanum(hardwareMap);
        m_drive.setPoseEstimate(new Pose2d(0, 0, 0));
    }

    public Mecanum getDrive(){
        return m_drive;
    }

    public void setPosition(Pose2d startPose){
        m_drive.setPoseEstimate(startPose);
        this.startPose = startPose;
    }

    public Pose2d getStartPose(){
        return startPose;
    }
}
