package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.teamcode.auto.commands.DetectProp;
import org.firstinspires.ftc.teamcode.auto.commands.DriveToBoard;
import org.firstinspires.ftc.teamcode.auto.commands.DriveToProp;
import org.firstinspires.ftc.teamcode.auto.commands.DriveToStack;
import org.firstinspires.ftc.teamcode.auto.commands.Park;
import org.firstinspires.ftc.teamcode.auto.commands.PushPixel;
import org.firstinspires.ftc.teamcode.bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.bot.commands.MoveSlideToPosition;
import org.firstinspires.ftc.teamcode.bot.commands.PlacePixel;
import org.firstinspires.ftc.teamcode.bot.commands.RetractLinearSlide;
import org.firstinspires.ftc.teamcode.bot.subsystems.CenterstageVision;
import org.firstinspires.ftc.teamcode.drive.FieldConstants;
import org.firstinspires.ftc.teamcode.opencv.pipeline.CenterStagePipeline;

@Autonomous
public class AutonomousMain extends LinearOpMode {

    CenterStageBot bot;

    DetectProp detectProp;
    DriveToProp driveToProp;
    PushPixel pushPixel;
    DriveToBoard driveToBoard;
    PlacePixel placePixel;
    DriveToStack driveToStack;
    Park park;
    InstantCommand close;

    CenterstageVision cv;

    FieldConstants.Stage stage = FieldConstants.Stage.BACK;
    FieldConstants.Side side = FieldConstants.Side.RED;

    private static Pose2d storedPose = new Pose2d(0, 0, 0);
    private static FieldConstants.Side storedSide = FieldConstants.Side.RED;

    @Override
    public void runOpMode() {
        bot = new CenterStageBot(hardwareMap);

        cv = new CenterstageVision(
                hardwareMap,
                "FrontCam",
                "RearCam"
        );
        cv.enablePipelineProcessor(true);

        detectProp = new DetectProp(cv);
        pushPixel = new PushPixel(bot.intake);
        placePixel = new PlacePixel(bot.slide, bot.clawArm, 1200);
        park = new Park(bot.drive, () -> side, () -> stage);
        close = new InstantCommand(bot.clawArm::close);

        while(!isStarted() && !isStopRequested()){
            if(gamepad1.dpad_up) stage = FieldConstants.Stage.BACK;
            else if(gamepad1.dpad_down) stage = FieldConstants.Stage.FRONT;

            if(gamepad1.dpad_left) side = FieldConstants.Side.BLUE;
            else if(gamepad1.dpad_right) side = FieldConstants.Side.RED;
            updateTelemetry();
        }

        cv.extractColorChannel(side == FieldConstants.Side.RED ? 1 : 2);
        bot.drive.setPosition(FieldConstants.getFieldStartPose(side, stage));

        driveToProp = new DriveToProp(bot.drive, cv, side, stage);
        driveToBoard = new DriveToBoard(bot.drive, cv, side, stage);
        driveToStack = new DriveToStack(bot.drive, bot.intake, side);

        CommandScheduler.getInstance().schedule(
                new RunCommand(telemetry::update),
                new SequentialCommandGroup(
                        close,
                        detectProp.withTimeout(500),
                        driveToProp,
                        pushPixel.withTimeout(400),
                        new ParallelCommandGroup(
                                new MoveSlideToPosition(bot.slide, 1200),
                                driveToBoard
                        ),
                        placePixel,
                        new ParallelCommandGroup(
                                driveToStack,
                                bot.retractSlide
                        ),
                        close,
                        new WaitCommand(300),
                        new MoveSlideToPosition(bot.slide, 1800),
                        new PlacePixel(bot.slide, bot.clawArm, 1800),
                        bot.retractSlide,
                        park
                )
        );

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            bot.drive.getDrive().update();
            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().reset();

        storedSide = side;
    }

    public static void storePosition(Pose2d pose){
        storedPose = pose;
    }

    public void updateTelemetry() {
        telemetry.addData("Position", stage + " " + side);
        telemetry.update();
    }

    public static Pose2d getStoredPose(){
        return storedPose;
    }

    public static FieldConstants.Side getStartSide(){
        return storedSide;
    }
}
