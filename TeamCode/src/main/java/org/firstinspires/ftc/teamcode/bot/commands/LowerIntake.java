package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;

public class LowerIntake extends CommandBase {

    private final Intake m_intake;

    public LowerIntake(Intake intake){
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setState(Intake.IntakeState.LOWERED);
        m_intake.power();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
