package org.firstinspires.ftc.teamcode.util.servo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTester extends LinearOpMode {

    private Servo servo;
    public static String NAME = "claw";
    public static double POSITION = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, NAME);
        servo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            servo.setPosition(POSITION);
        }
    }
}
