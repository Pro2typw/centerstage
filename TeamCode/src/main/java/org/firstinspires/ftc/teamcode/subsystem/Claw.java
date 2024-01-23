package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.justbetter.actuator.CachingServo;
import org.firstinspires.ftc.teamcode.subsystem.util.Constants;
import org.jetbrains.annotations.NotNull;

public class Claw {
    public static enum ClawState {
        OPEN,
        CLOSE
    }
    public static enum ClawSide {
        LEFT,
        RIGHT,
        BOTH
    }

    private final CachingServo rightClaw, leftClaw;
    private final DistanceSensor rightSensor, leftSensor;

    private ClawState rightClawState, leftClawState;

    private boolean flipped;
    
    public Claw(@NotNull HardwareMap hardwareMap, @NotNull ClawState state) {
        rightClaw = new CachingServo(hardwareMap.get(Servo.class, Constants.Claw.RIGHT_CLAW_MAP_NAME));
        leftClaw = new CachingServo(hardwareMap.get(Servo.class, Constants.Claw.LEFT_CLAW_MAP_NAME));

//        rightSensor = hardwareMap.get(DistanceSensor.class, Constan);
//        leftSensor = hardwareMap.get(DistanceSensor.class, "..");

        setClawState(ClawSide.BOTH, state);
        flipped = false;
    }
    
    public void setClawState(@NotNull ClawSide side, @NotNull ClawState state) {
        if(!flipped) {
            switch(side) {
                case LEFT:
                    leftClawState = state;
                    leftClaw.setPosition(state == ClawState.CLOSE ? Constants.Claw.LEFT_CLAW_CLOSE_POSITION : Constants.Claw.LEFT_CLAW_OPEN_POSITION);
                    break;
                case BOTH:
                    leftClawState = state;
                    leftClaw.setPosition(state == ClawState.CLOSE ? Constants.Claw.LEFT_CLAW_CLOSE_POSITION : Constants.Claw.LEFT_CLAW_OPEN_POSITION);
                case RIGHT:
                    rightClawState = state;
                    rightClaw.setPosition(state == ClawState.CLOSE ? Constants.Claw.RIGHT_CLAW_CLOSE_POSITION : Constants.Claw.RIGHT_CLAW_OPEN_POSITION);
                    break;
            }
        }
        else {
            switch(side) {
                case RIGHT:
                    leftClawState = state;
                    leftClaw.setPosition(state == ClawState.CLOSE ? Constants.Claw.LEFT_CLAW_CLOSE_POSITION : Constants.Claw.LEFT_CLAW_OPEN_POSITION);
                    break;
                case BOTH:
                    leftClawState = state;
                    leftClaw.setPosition(state == ClawState.CLOSE ? Constants.Claw.LEFT_CLAW_CLOSE_POSITION : Constants.Claw.LEFT_CLAW_OPEN_POSITION);
                case LEFT:
                    rightClawState = state;
                    rightClaw.setPosition(state == ClawState.CLOSE ? Constants.Claw.RIGHT_CLAW_CLOSE_POSITION : Constants.Claw.RIGHT_CLAW_OPEN_POSITION);
                    break;
            }
        }

    }

    public ClawState getClawState(@NotNull ClawSide side) {
        if(side == ClawSide.LEFT) return leftClawState;
        else if(side == ClawSide.RIGHT) return  rightClawState;
        else return leftClawState == rightClawState ? leftClawState : ClawState.CLOSE;
    }

    public void toggleClawState() {
        setClawState(ClawSide.BOTH, getClawState(ClawSide.BOTH) == ClawState.CLOSE ? ClawState.OPEN : ClawState.CLOSE);
    }

    public void setFlipped(boolean flipped) {
        this.flipped = flipped;
    }

}