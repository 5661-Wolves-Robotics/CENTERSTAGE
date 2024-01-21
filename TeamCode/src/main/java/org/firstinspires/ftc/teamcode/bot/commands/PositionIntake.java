package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;

public class PositionIntake extends CommandBase {

    private final Intake m_intake;
    private final double POS;

    public PositionIntake(Intake intake, double pos){
        m_intake = intake;
        POS = pos;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setPos(POS);
        m_intake.power();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
