package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
public class Transfer {
    public static class Params {
        public int transferInPwm = 679, transferShootPwm = 920;
        public int transferStopperDownPwm = 1198, transferStopperUpPwm = 1357;
        public double transferMoveTime = 0.2, transferStopperMoveTime = 0.1, transferResetTime = 0.05;
        public boolean autoLockHood = true;
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
    private boolean shouldShootAll, shouldShootSafely;
    public ArrayList<Double> vels, hoods;
    public Transfer(Robot robot) {
        vels = new ArrayList<>();
        hoods = new ArrayList<>();
        this.robot = robot;

        transfer = robot.hardwareMap.get(ServoImplEx.class, "transfer");
        transfer.setPwmRange(new PwmControl.PwmRange(params.transferInPwm, params.transferShootPwm));
        transfer.setPosition(0);

        transferStopper = robot.hardwareMap.get(ServoImplEx.class, "transferStopper");
        transferStopper.setPwmRange(new PwmControl.PwmRange(params.transferStopperDownPwm, params.transferStopperUpPwm));
        transferStopper.setPosition(0);

        transferTimer = new ElapsedTime();
        transferState = TransferState.OFF;
        shouldShootSafely = false;
    }
    public void update() {
        switch(transferState) {
            case OFF:
                if((robot.g1.rightBumper() || shouldShootAll)
                && robot.indexer.prettyMuchStaticShoot()
                && (!shouldShootSafely || (robot.shooter.getShooterVelocity() > robot.shooter.getMinShooterVel() && robot.shooter.getShooterVelocity() < robot.shooter.getMaxShooterVel())))
                    setTransferTransferring();
                break;
            case TRANSFERRING:
                if(transferTimer.seconds() > params.transferStopperMoveTime && transfer.getPosition() < 0.5) {
                    transfer.setPosition(0.99);
                    if(!robot.indexer.emptyAt(robot.indexer.getShooterI()))
                        robot.indexer.simulateShot();
                    if(params.autoLockHood)
                        robot.shooter.setHoodLocked(true);
                    vels.add(robot.shooter.getShooterVelocity());
                    hoods.add(Math.floor(robot.shooter.getHoodPos()*100000)/100000);
                }
                if(transferTimer.seconds() > params.transferStopperMoveTime + params.transferMoveTime) {
                    setTransferResetting();
                    if(robot.indexer.getNumBalls() > 0)
                        robot.indexer.rotate(robot.indexer.getAlignShootOffset());
                    else
                        robot.indexer.rotate(robot.indexer.getAlignCollectOffset());
                }
                break;
            case RESETTING:
                if(transferTimer.seconds() > params.transferResetTime) {
                    setTransferOff();
//                    if(robot.indexer.getNumBalls() == 0)
//                        robot.shooter.setResting(true);
                    robot.shooter.setHoodLocked(false);
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
    }
    private void setTransferOff() {
        transferState = TransferState.OFF;
        transferStopper.setPosition(0);
    }
    public TransferState getTransferState() {
        return transferState;
    }
    public void setShootAll(boolean shootAll) {
        this.shouldShootAll = shootAll;
    }
    public boolean shouldShootAll() {
        return shouldShootAll;
    }
    public void setShootSafely(boolean safely) {
        shouldShootSafely = safely;
    }
}
