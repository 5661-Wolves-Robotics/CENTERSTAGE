package org.firstinspires.ftc.teamcode.bot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;

public class RaiseIntake extends CommandBase {

    private final Intake m_intake;

    public RaiseIntake(Intake intake){
        m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setState(Intake.IntakeState.RAISED);
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
