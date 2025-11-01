package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transfer {
    public static class Params {
        public int transferInPwm = 620, transferShootPwm = 850;
        public int transferStopperDownPwm = 2310, transferStopperUpPwm = 2450;
        public double transferMoveTime = 0.13, transferStopperMoveTime = 0.1, transferResetTime = 0.05;
    }
    public static Params params = new Params();

    public final Robot robot;
    // transfer stuff
    private final ServoImplEx transfer;
    private final ServoImplEx transferStopper;
    private final ElapsedTime transferTimer; // tracks time spent transferring
    public enum TransferState {
        OFF, RESETTING, TRANSFERRING
    }
    private TransferState transferState;
    private boolean shouldShootAll;

    public Transfer(Robot robot) {
        this.robot = robot;

        transfer = robot.hardwareMap.get(ServoImplEx.class, "transfer");
        transfer.setPwmRange(new PwmControl.PwmRange(params.transferInPwm, params.transferShootPwm));
        transfer.setPosition(0);

        transferStopper = robot.hardwareMap.get(ServoImplEx.class, "transferStopper");
        transferStopper.setPwmRange(new PwmControl.PwmRange(params.transferStopperDownPwm, params.transferStopperUpPwm));
        transferStopper.setPosition(0);

        transferTimer = new ElapsedTime();
        transferState = TransferState.OFF;
    }
    public void update() {
        switch(transferState) {
            case OFF:
                if((robot.g1.rightBumper() || shouldShootAll) && robot.indexer.prettyMuchStatic())
                    setTransferTransferring();
                break;
            case TRANSFERRING:
                if(transferTimer.seconds() > params.transferStopperMoveTime && transfer.getPosition() < 0.5) {
                    transfer.setPosition(0.99);
                    if(!robot.indexer.emptyAt(robot.indexer.getShooterI())) {
                        robot.indexer.simulateShot();
                    }
                }
                if(transferTimer.seconds() > params.transferStopperMoveTime + params.transferMoveTime)
                    setTransferResetting();
                break;
            case RESETTING:
                if(transferTimer.seconds() > params.transferResetTime) {
                    setTransferOff();
                    robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
                }
        }
    }
    private void setTransferTransferring() {
        transferState = TransferState.TRANSFERRING;
        transferTimer.reset();
        transferStopper.setPosition(0.99);
    }
    private void setTransferResetting() {
        transferState = TransferState.RESETTING;
        transferTimer.reset();
        transfer.setPosition(0);
        transferStopper.setPosition(0);
    }
    private void setTransferOff() {
        transferState = TransferState.OFF;
    }
    public TransferState getTransferState() {
        return transferState;
    }
    public void setShootAll(boolean shootAll) {
        this.shouldShootAll = shootAll;
    }
}
