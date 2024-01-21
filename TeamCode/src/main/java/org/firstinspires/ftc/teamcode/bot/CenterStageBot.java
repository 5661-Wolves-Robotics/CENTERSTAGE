package org.firstinspires.ftc.teamcode.bot;

import static org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm.ArmState.LOWERED;
import static org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm.ArmState.RAISED;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bot.commands.AutoLowerArm;
import org.firstinspires.ftc.teamcode.bot.commands.MoveArm;
import org.firstinspires.ftc.teamcode.bot.commands.RetractLinearSlide;
import org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm;
import org.firstinspires.ftc.teamcode.bot.subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.bot.subsystems.DualLinearSlide;
import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.bot.subsystems.MecanumDriveBase;

@Config
public class CenterStageBot extends Robot {

    public MecanumDriveBase drive;
    public ClawArm clawArm;
    public DualLinearSlide slide;
    public Intake intake;
    public DroneLauncher droneLauncher;

    public RetractLinearSlide retractSlide;
    public MoveArm lowerArm;
    public MoveArm raiseArm;
    private AutoLowerArm autoLowerArm;

    public CenterStageBot(HardwareMap hardwareMap){
        drive = new MecanumDriveBase(hardwareMap);
        slide = new DualLinearSlide(hardwareMap, "rightSlide", "leftSlide", 4300);
        clawArm = new ClawArm(hardwareMap, "arm", "claw", slide::getPosition);
        intake = new Intake(hardwareMap, "dropdown", "perpendicularEncoder");
        droneLauncher = new DroneLauncher(hardwareMap, "launcher");

        retractSlide = new RetractLinearSlide(slide, clawArm);

        raiseArm = new MoveArm(clawArm, RAISED);
        lowerArm = new MoveArm(clawArm, LOWERED);
        autoLowerArm = new AutoLowerArm(slide, clawArm);

        register(clawArm);
    }

}
