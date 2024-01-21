package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutonomousMain;
import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.bot.commands.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.bot.commands.LocalDrive;
import org.firstinspires.ftc.teamcode.bot.commands.PlacerDrive;
import org.firstinspires.ftc.teamcode.bot.commands.PowerSlide;
import org.firstinspires.ftc.teamcode.bot.commands.ToggleIntake;
import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.bot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;

@TeleOp
public class TeleOpMain extends LinearOpMode {

    private static final int RUMBLE_DUR = 800;

    GamepadEx driver, placer;
    LocalDrive localDrive;
    PlacerDrive placerDrive;
    PowerSlide powerSlide;
    ToggleIntake toggleIntake;

    boolean colliding = false;

    @Override
    public void runOpMode() {
        CenterStageBot bot = new CenterStageBot(hardwareMap);

        CenterstageVision cv = new CenterstageVision(
                hardwareMap,
                "FrontCam",
                "RearCam"
        );

        driver = new GamepadEx(gamepad1);
        placer = new GamepadEx(gamepad2);

        localDrive = new LocalDrive(bot.drive, driver::getLeftX, driver::getLeftY, driver::getRightX, ()->colliding);
        placerDrive = new PlacerDrive(bot.drive, placer::getLeftX, placer::getLeftY, placer::getRightX, AutonomousMain.getStartSide());

        powerSlide = new PowerSlide(bot.slide, () -> placer.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - placer.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        toggleIntake = new ToggleIntake(bot.intake);

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(toggleIntake);
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(bot.intake::reverse, bot.intake))
                .whenReleased(new ConditionalCommand(
                        new InstantCommand(bot.intake::power, bot.intake),
                        new InstantCommand(bot.intake::stop, bot.intake),
                        ()-> bot.intake.getState() == Intake.IntakeState.LOWERED
                ));
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(placerDrive)
                .whenPressed(new InstantCommand(bot.clawArm::close, bot.clawArm))
                .whenPressed(() -> gamepad2.rumble(RUMBLE_DUR));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> colliding = !colliding);

        placer.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(bot.raiseArm);
        placer.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(bot.lowerArm);
        placer.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(bot.droneLauncher::launch, bot.droneLauncher));
        placer.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(
                        new SequentialCommandGroup(
                            new InstantCommand(bot.clawArm::open, bot.clawArm),
                            new WaitCommand(200),
                            bot.retractSlide
                        ),
                        new InstantCommand(bot.clawArm::toggleClaw, bot.clawArm),
                        () -> placerDrive.isScheduled()
                ))
                .whenPressed(() -> {
                    if(placerDrive.isScheduled()) gamepad1.rumble(RUMBLE_DUR);
                })
                .cancelWhenPressed(placerDrive);

        CommandScheduler.getInstance().registerSubsystem(bot.drive, bot.slide);

        bot.drive.setDefaultCommand(localDrive);
        bot.slide.setDefaultCommand(powerSlide);

        CommandScheduler.getInstance().schedule(
                new RunCommand(telemetry::update),
                new AprilTagLocalization(cv, bot.drive)
        );

        bot.drive.setPosition(AutonomousMain.getStoredPose());

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            bot.drive.getDrive().update();
            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().reset();
    }
}
