package org.firstinspires.ftc.teamcode.rr.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.WPIMathUtil;

public class ArmC {
    Robot robot;
    public ArmC(Robot robot) {
        this.robot = robot;
    }

    private class ToDeposit implements Action {
        public boolean passedTransit;
        public ToDeposit() {
            passedTransit = robot.arm.getPivotCurrentPos() > 150;
            robot.arm.setPivotTargetPos(580);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(passedTransit) robot.wrist.setPosition(Constants.Wrist.DEPO_POS_580);
            else robot.wrist.setPosition(Constants.Wrist.INTAKE_POS);

            return !WPIMathUtil.isNear(580, robot.arm.getPivotCurrentPos(), 20);
        }
    }

    public Action toDeposit() {
        return new ToDeposit();
    }

    private class SetClawState implements Action {
        Claw.ClawSide side;
        Claw.ClawState state;

        public SetClawState(Claw.ClawSide side, Claw.ClawState state) {
            this.side = side;
            this.state = state;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.claw.setClawState(side, state);
            return false;
        }
    }

    public Action setClawState(Claw.ClawSide side, Claw.ClawState state) {
        return new SetClawState(side, state);
    }

    // WRIST
    private class SetWristPosition implements Action {
        double wrist;

        public SetWristPosition(double wrist) {
            this.wrist = wrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.wrist.setPosition(wrist);
            return false;
        }
    }

    public Action setWristPosition(double wrist) {
        return new SetWristPosition(wrist);
    }

    // PIVOT
    private class SetPivotPosition implements Action {
        double pos;

        public SetPivotPosition(double pivot) {
            this.pos = pivot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.wrist.setPosition(pos);
            return false;
        }
    }

    public Action setPivotPosition(double pos) {
        return new SetPivotPosition(pos);
    }

    // EXTENSION
    private class SetExtensionPosition implements Action {
        double extend;

        public SetExtensionPosition(double pos) {
            this.extend = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.wrist.setPosition(extend);
            return false;
        }
    }

    public Action setExtensionPosition(double pos) {
        return new SetExtensionPosition(pos);
    }

    private class WaitSeconds implements Action {
        double currentTime;
        double targetTime;
        public WaitSeconds(double ms) {
            currentTime = System.currentTimeMillis();
            targetTime = currentTime + ms;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return currentTime < targetTime;
        }
    }

    public Action waitSeconds(double ms) {
        return new WaitSeconds(ms);
    }

    private class Update implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.update();
            return true;
        }
    }

    public Action update() {
        return new Update();
    }

}
