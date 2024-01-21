package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.bot.subsystems.ClawArm;
import org.firstinspires.ftc.teamcode.bot.subsystems.DualLinearSlide;

public class PlacePixel extends SequentialCommandGroup {
    public PlacePixel(DualLinearSlide slide, ClawArm clawArm, int height){
        addCommands(
                new MoveArm(clawArm, ClawArm.ArmState.LOWERED),
                new WaitCommand(300),
                new InstantCommand(clawArm::open, clawArm),
                new WaitCommand(300)
        );
        addRequirements(slide, clawArm);
    }

}