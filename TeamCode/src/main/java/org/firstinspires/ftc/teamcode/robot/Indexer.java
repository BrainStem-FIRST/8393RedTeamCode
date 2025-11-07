package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall.BallColor;

@Config
public class Indexer {
    public static class Params {
        public double manualIndexPowerAmp = 0.1;
        public int greenPos = 0; // ranges 0-2 (0=green should be shot first, 2=green should be shot last)
        public double csResponseDelay = 0;
        public double kPClockwise60 = 0.00055, kPCounter60 = 0.0006, kPClockwise120 = 0.00065, kPCounter120 = 0.0007, kPClockwise180 = 0.0005;
        public double kPClockwise120Auto = 0.00065, kPCounter120Auto = 0.0007;
        public double kI = 0, kD = 0, kF = 0.01, minPower = 0.085, autoMinPower = 0.095;
        public double thirdRotateAmount = 2733.333333; // encoders needed to rotate 120 degrees
        public double oscillateAmount = 50, oscillatePower = 0.1;
        public double shootStaticVelThreshold = 500, shootStaticEncoderThreshold = 200;
        public double csStaticVelThreshold = 1000, csStaticEncoderThreshold = 200;
        public int errorThreshold = 75;
        public double lightFlashTime = 0.3; // time that light flashes for after each color sensor recognition
        public double timeSinceLastRotateThreshold = 1; // after each rotation, i will recheck all valid color sensors for this amount of time
    }
    public static Params params = new Params();

    private final Robot robot;
    // indexer stuff
    private final CRServo indexer;
    private final DcMotorEx indexerEncoderTracker;
    private int curEncoder;
    private boolean encoderUpdated; // for caching
    private double targetEncoder, oscillateTargetEncoder;
    private final PIDFController cwPid60, ccwPid60, cwPid120, ccwPid120, ccPid180;
    private final PIDFController cwPid120Auto, ccwPid120Auto;
    private PIDFController indexerPid;
    private String pidSelected;
    private double indexPower, prevIndexPower;

    // state stuff
    public enum IndexerState {
        TARGET, OSCILLATE
    }
    private IndexerState indexerState;
    private boolean shouldOscillate;

    // misc logic stuff
    private final BallColor[] ballList;
    private int intakeI; // 0-5: represents 6 possible places where ball could be in the indexer; index 0 is index of central intake spot, indexes increase in clockwise order
    private int numBalls;
    private final ColorSensorBall leftCS, rightCS, midCS; // color sensors
    private boolean shouldCheckLeftCS, shouldCheckRightCS, shouldCheckMidCS;
    private boolean ballAtLeft, ballAtRight, ballAtMid;
    private int curPatternI; // current pattern color selected
    private boolean shouldAutoRotate, autoRotateCued;
    private final ElapsedTime indexerAutoRotateTimer, timeSinceLastRotate;
    private BallColor lastIntakedColor;
    private double intakeOffset;
    private boolean inAutonomous;

    public Indexer(Robot robot) {
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "indexer");

        indexPower = 0;
        indexerEncoderTracker = robot.hardwareMap.get(DcMotorEx.class, "FL");
        resetIndexerEncoder();
        targetEncoder = 0;
        cwPid60 = new PIDFController(new PIDFCoefficients(params.kPClockwise60, params.kI, params.kD, params.kF));
        ccwPid60 = new PIDFController(new PIDFCoefficients(params.kPCounter60, params.kI, params.kD, params.kF));
        cwPid120 = new PIDFController(new PIDFCoefficients(params.kPClockwise120, params.kI, params.kD, params.kF));
        ccwPid120 = new PIDFController(new PIDFCoefficients(params.kPCounter120, params.kI, params.kD, params.kF));
        ccPid180 = new PIDFController(new PIDFCoefficients(params.kPClockwise180, params.kI, params.kD, params.kF));
        cwPid120Auto = new PIDFController(new PIDFCoefficients(params.kPClockwise120Auto, params.kI, params.kD, params.kF));
        ccwPid120Auto = new PIDFController(new PIDFCoefficients(params.kPCounter120Auto, params.kI, params.kD, params.kF));
        indexerPid = new PIDFController(new PIDFCoefficients(params.kPClockwise60, params.kI, params.kD, params.kF));

        autoRotateCued = false;
        indexerAutoRotateTimer = new ElapsedTime();
        timeSinceLastRotate = new ElapsedTime();

        ballList = new BallColor[] {BallColor.N, BallColor.N, BallColor.N, BallColor.N, BallColor.N, BallColor.N};
        intakeI = 0;
        numBalls = 0;

        leftCS = new ColorSensorBall(robot, "leftCS");
        rightCS = new ColorSensorBall(robot, "rightCS");
        midCS = new ColorSensorBall(robot, "midCS");

        shouldCheckLeftCS = true;
        shouldCheckRightCS = true;
        shouldCheckMidCS = true;

        curPatternI = 0;
        shouldAutoRotate = true;
        pidSelected = "normal";

        lastIntakedColor = BallColor.N;
        indexerState = IndexerState.TARGET;
        shouldOscillate = true;
    }
    public void resetCaches() {
        encoderUpdated = false;
    }

    public void update() {
        updateIndexer(updateColorSensors());
        updateIndexerAutoRotate();
    }
    private boolean updateColorSensors() {
        leftCS.update();
        rightCS.update();
        midCS.update();
        // updating color sensor values
        boolean oscillateValid = robot.intake.getIntakeState() != Intake.IntakeState.OFF || timeSinceLastRotate.seconds() < params.timeSinceLastRotateThreshold;
        if (shouldAutoRotate && prettyMuchStaticCS() && oscillateValid) {
            // potentially check middle sensor if indexer at correct offset
            if(intakeI % 2 == intakeOffset) {
                if (shouldCheckMidCS && emptyAt(intakeI) && midCS.getBallColor() != BallColor.N) {
                    ballAtMid = true;
                    ballAtLeft = false;
                    ballAtRight = false;
                    ballList[intakeI] = midCS.getBallColor();
                    numBalls++;
                    indexerAutoRotateTimer.reset();
                    autoRotateCued = true;
                    lastIntakedColor = midCS.getBallColor();
                }
            }
            // potentially check left and right sensors if indexer is at correct offset
            else {
                boolean ballAtLeft = shouldCheckLeftCS && emptyAt(getLeftIntakeI()) && leftCS.getBallColor() != BallColor.N;
                if (ballAtLeft) {
                    ballList[getLeftIntakeI()] = leftCS.getBallColor();
                    numBalls++;
                    this.ballAtLeft = true;
                    lastIntakedColor = leftCS.getBallColor();
                }
                boolean ballAtRight = shouldCheckRightCS && emptyAt(getRightIntakeI()) && rightCS.getBallColor() != BallColor.N;
                if (ballAtRight) {
                    ballList[getRightIntakeI()] = rightCS.getBallColor();
                    numBalls++;
                    this.ballAtRight = true;
                    lastIntakedColor = rightCS.getBallColor();
                }
                if (ballAtLeft || ballAtRight) {
                    indexerAutoRotateTimer.reset();
                    autoRotateCued = true;
                }
                ballAtMid = false;
            }
            if(autoRotateCued)
                robot.rgbLight.setState(RGBLight.LightState.SET, lastIntakedColor == BallColor.G ? RGBLight.params.green : RGBLight.params.purple, params.lightFlashTime);
        }
        return oscillateValid;
    }
    private void updateIndexer(boolean oscillateValid) {
        // listening for gamepad input to index
        if (robot.transfer.getTransferState() == Transfer.TransferState.OFF) {
            if (robot.g1.isFirstB())
                rotate(1);
            else if (robot.g1.isFirstA())
                rotate(-1);
        }

        // listen for oscillate state change
        if(shouldOscillate && oscillateValid && emptyAt(intakeI) && intakeI % 2 == intakeOffset) {
            if (indexerState == IndexerState.TARGET) {
                indexerState = IndexerState.OSCILLATE;
                oscillateTargetEncoder = targetEncoder + params.oscillateAmount;
            }
        }
        // listen for target state change
        else if(indexerState == IndexerState.OSCILLATE)
            indexerState = IndexerState.TARGET;


        // calculating indexer power
        if(indexerState == IndexerState.TARGET) {
            if (Math.abs(getIndexerError()) < params.errorThreshold)
                indexPower = 0;
            else {
                if (robot.g2.rightTrigger() > 0.05)
                    indexPower = Math.signum(getIndexerError()) * robot.g2.rightTrigger() * params.manualIndexPowerAmp;
                else {
                    calcIndexerPidPower();
                }
            }
        }
        else if(indexerState == IndexerState.OSCILLATE) {
            // updating oscillate goal position
            double dir = Math.signum(oscillateTargetEncoder - targetEncoder);
            if(dir != Math.signum(oscillateTargetEncoder - getIndexerEncoder())) {
                dir *= -1;
                oscillateTargetEncoder = targetEncoder + params.oscillateAmount * dir;
            }

            if(Math.abs(oscillateTargetEncoder - getIndexerEncoder()) > params.oscillateAmount * 2)
                calcIndexerPidPower();
            else
                indexPower = params.oscillatePower * -dir;
        }

        if(indexPower != prevIndexPower)
            indexer.setPower(indexPower);
        prevIndexPower = indexPower;

        //misc
        if(robot.g1.isFirstStart())
            shouldOscillate = !shouldOscillate;
    }
    private void calcIndexerPidPower() {
        indexerPid.updatePosition(getIndexerEncoder());
        indexPower = indexerPid.run();
        indexPower = Math.max(Math.abs(indexPower), inAutonomous ? params.autoMinPower : params.minPower) * Math.signum(indexPower) * -1;
    }
    private void updateIndexerAutoRotate() {
        if(!shouldAutoRotate || !autoRotateCued || indexerAutoRotateTimer.seconds() < params.csResponseDelay)
            return;
        if (numBalls == 3) {
            rotate(getAlignIndexerOffset());
            robot.shooter.setResting(false);
        }
        else if (ballAtLeft && ballAtRight)
            //rotating 180 degrees clockwise
            rotate(3);
        else if (ballAtLeft)
            if (numBalls == 1)
                // rotating 120 degrees clockwise
                rotate(-2);
            else if(!emptyAt(getShooterI()))
                // rotating 60 degrees clockwise
                rotate(-1);
            else
                // rotating 180 degrees clockwise
                rotate(-3);
        else if(ballAtRight) {
            if (numBalls == 1)
                // rotating 120 degrees counter clockwise
                rotate(2);
            else if(!emptyAt(getShooterI()))
                // rotating 60 degrees counter clockwise
                rotate(1);
            else
                // rotating 180 degrees clockwise
                rotate(-3);
        }

        else if(ballAtMid) {
            if(!emptyAt(getOffsetI(getShooterI(), 1)))
                // rotating 120 degrees clockwise
                rotate(-2);
            else
                rotate(2);
        }
        autoRotateCued = false;
        ballAtLeft = false;
        ballAtMid = false;
        ballAtRight = false;
    }
    private void resetIndexerEncoder() {
        indexerEncoderTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexerEncoderTracker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    // returns offset to rotate indexer to align it to next pattern color (pos=clockwise, neg=ccw, -10=no valid pattern color)
    public int getAlignIndexerOffset() {
        if(numBalls == 0)
            return 1;
        BallColor desiredColor = curPatternI == Robot.params.greenPos ? BallColor.G : BallColor.P;
        int desiredI = findBallI(desiredColor);
        if(desiredI == -1) // finding index of other color ball if there is no balls of correct color
            desiredI = findBallI(desiredColor == BallColor.G ? BallColor.P : BallColor.G);
        int disp = getShooterI() - desiredI;
        if(disp > 2)
            disp -= 6;
        else if(disp < -3)
            disp += 6;
        return disp;
    }
    private PIDFController calcRotationPid(int sixth) {
        if(sixth == 1) {
            pidSelected = "counter clockwise 60";
            return ccwPid60;
        }
        else if(sixth == -1) {
            pidSelected = "clockwise 60";
            return cwPid60;
        }
        else if(sixth == 2) {
            pidSelected = "counter clockwise 120";
            if(inAutonomous) {
                pidSelected += " AUTO";
                return ccwPid120Auto;
            }
            return ccwPid120;
        }
        else if(sixth == -2) {
            pidSelected = "clockwise 120";
            if(inAutonomous) {
                pidSelected += " AUTO";
                return cwPid120Auto;
            }
            return cwPid120;
        }
        pidSelected = "clockwise 180";
        return ccPid180;
    }
    // positive value = counter clockwise rotation
    public void rotate(int sixth) {
        intakeI = (intakeI - sixth + 6) % 6;
        targetEncoder += sixth * params.thirdRotateAmount/2;
        indexerPid = calcRotationPid(sixth);
        indexerPid.setTargetPosition(targetEncoder);

        shouldCheckMidCS = emptyAt(intakeI);
        shouldCheckLeftCS = emptyAt(getLeftIntakeI());
        shouldCheckRightCS = emptyAt(getRightIntakeI());
        timeSinceLastRotate.reset();
    }
    public void simulateShot() {
        if(!emptyAt(getShooterI())) {
            numBalls = Math.max(0, numBalls - 1);
            ballList[getShooterI()] = ColorSensorBall.BallColor.N;
        }
        curPatternI = (curPatternI + 1) % 3;
    }
    public int getIndexerEncoder() {
        if(!encoderUpdated) {
            encoderUpdated = true;
            curEncoder = indexerEncoderTracker.getCurrentPosition();
        }
        return curEncoder;
    }
    public double getIndexerPower() {
        return indexer.getPower();
    }
    public int getNumBalls() {
        return numBalls;
    }
    public double getTargetEncoder() {
        return targetEncoder;
    }
    public double getIndexerError() {
        return targetEncoder - getIndexerEncoder();
    }
    public BallColor getLeftCSColor() {
        return leftCS.getBallColor();
    }
    public BallColor getRightCSColor() {
        return rightCS.getBallColor();
    }
    public BallColor getMidCSColor() {
        return midCS.getBallColor();
    }
    public boolean prettyMuchStaticShoot() {
        return indexerEncoderTracker.getVelocity() < params.shootStaticVelThreshold && Math.abs(getIndexerError()) < params.shootStaticEncoderThreshold;
    }
    public boolean prettyMuchStaticCS() {
        return indexerEncoderTracker.getVelocity() < params.csStaticVelThreshold && Math.abs(getIndexerError()) < params.csStaticEncoderThreshold;

    }
    public double getIndexerVel() {
        return indexerEncoderTracker.getVelocity();
    }
    public boolean emptyAt(int i) {
        return ballList[i] == BallColor.N;
    }
    public int getIntakeI() {
        return intakeI;
    }
    public int getLeftIntakeI() {
        return getOffsetI(intakeI, -1);
    }
    public int getRightIntakeI() {
        return getOffsetI(intakeI, 1);
    }
    public int getOffsetI(int i, int offset) {
        return (i + offset + 6) % 6;
    }
    public int getShooterI() {
        return (intakeI + 3) % 6;
    }
    public int findBallI(BallColor ballColor) {
        for(int i = 0; i < 4; i++) {
            int index = getOffsetI(getShooterI(), i);
            if(getBallAt(index) == ballColor)
                return index;
            index = getOffsetI(getShooterI(), -i);
            if(getBallAt(index) == ballColor)
                return index;
        }
        return -1;
    }
    public BallColor getBallAt(int i) {
        return ballList[i];
    }
    public boolean shouldCheckLeftCS() {
        return shouldCheckLeftCS;
    }
    public boolean shouldCheckRightCS() {
        return shouldCheckRightCS;
    }
    public boolean shouldCheckMidCS() {
        return shouldCheckMidCS;
    }
    public int getCurPatternI() {
        return curPatternI;
    }
    public String getLabeledBalls() {
        StringBuilder label = new StringBuilder();
        for(int i = 0; i < ballList.length; i++) {
            if(i == intakeI)
                label.append("i{").append(ballList[i]).append("}, ");
            else if(i == getShooterI())
                label.append("s[").append(ballList[i]).append("], ");
            else
                label.append(" ").append(ballList[i]).append(", ");
        }
        return label.toString();
    }
    public String getPidSelected() {
        return pidSelected;
    }
    public boolean isBallAtLeft() {
        return ballAtLeft;
    }
    public boolean isBallAtMid() {
        return ballAtMid;
    }
    public boolean isBallAtRight() {
        return ballAtRight;
    }
    public boolean isAutoRotateCued() {
        return autoRotateCued;
    }
    public boolean shouldAutoIndex() {
        return shouldAutoRotate;
    }
    public void setAutoRotate(boolean shouldAutoRotate) {
        this.shouldAutoRotate = shouldAutoRotate;
    }
    public void setAutoBallList(int offset, BallColor b1, BallColor b2, BallColor b3) {
        intakeOffset = offset;
        ballList[intakeI + offset] = b1;
        ballList[getOffsetI(intakeI + offset, 2)] = b2;
        ballList[getOffsetI(intakeI + offset, 4)] = b3;
        numBalls = 3;
    }
    public boolean shouldOscillate() {
        return shouldOscillate;
    }
    public void setShouldOscillate(boolean shouldOscillate) {
        this.shouldOscillate = shouldOscillate;
    }
    public double getOscillateTargetEncoder() {
        return oscillateTargetEncoder;
    }
    public IndexerState getIndexerState() {
        return indexerState;
    }
    public void setInAutonomous() {
        inAutonomous = true;
    }
    public BallColor getLastIntakedColor() {
        return lastIntakedColor;
    }
}
