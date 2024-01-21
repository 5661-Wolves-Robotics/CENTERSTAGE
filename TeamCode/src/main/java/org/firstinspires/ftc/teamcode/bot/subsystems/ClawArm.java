package org.firstinspires.ftc.teamcode.bot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.servo.ToggleServo;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class ClawArm extends SubsystemBase {

    private final Servo m_armServo;
    private final ToggleServo m_clawServo;
    private final IntSupplier m_slidePos;

    public enum ArmState{
        RAISED(0.38),
        STORED(0.295),
        LOWERED(0.21);

        private final double pos;
        ArmState(double pos){
            this.pos = pos;
        }

        double getValue(){
            return pos;
        }
    }

    private ArmState armState = ArmState.STORED;

    public ClawArm(HardwareMap hardwareMap, String armServo, String clawServo, IntSupplier slidePos){
        m_armServo = hardwareMap.get(Servo.class, armServo);
        m_clawServo = new ToggleServo(hardwareMap.get(Servo.class, clawServo), 0.17 /*OPEN*/, 0.11 /*CLOSED*/, true, true);
        m_slidePos = slidePos;

        setArmState(armState);
    }

    public void setArmState(ArmState state){
        armState = state;
        m_armServo.setPosition(state.getValue());
    }

    public ArmState getArmState(){
        return armState;
    }

    public void open(){
        m_clawServo.setState(true);
    }

    public void close(){
        m_clawServo.setState(false);
    }

    public void toggleClaw(){
        m_clawServo.toggle();
    }

    @Override
    public void periodic() {
        if(m_slidePos.getAsInt() <= 830){
            setArmState(ArmState.STORED);
        }
    }
}
