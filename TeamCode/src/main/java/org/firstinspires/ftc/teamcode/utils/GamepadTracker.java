package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.IntConsumer;
import java.util.function.IntSupplier;

public class GamepadTracker {
    private final Gamepad gamepad;
    private int aFrameCount = 0;
    private int bFrameCount = 0;
    private int xFrameCount = 0;
    private int yFrameCount = 0;
    private int dpadUpFrameCount = 0;
    private int dpadDownFrameCount = 0;
    private int dpadLeftFrameCount = 0;
    private int dpadRightFrameCount = 0;
    private int leftBumperFrameCount = 0;
    private int rightBumperFrameCount = 0;
    private int leftTriggerFrameCount = 0;
    private int rightTriggerFrameCount = 0;
    private int startButtonFrameCount = 0;
    private int backButtonFrameCount = 0;

    public GamepadTracker(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        // Update each button's frame count
        updateButtonFrame(gamepad.a, () -> aFrameCount, (c) -> aFrameCount = c);
        updateButtonFrame(gamepad.b, () -> bFrameCount, (c) -> bFrameCount = c);
        updateButtonFrame(gamepad.x, () -> xFrameCount, (c) -> xFrameCount = c);
        updateButtonFrame(gamepad.y, () -> yFrameCount, (c) -> yFrameCount = c);

        updateButtonFrame(gamepad.dpad_up, () -> dpadUpFrameCount, (c) -> dpadUpFrameCount = c);
        updateButtonFrame(gamepad.dpad_down, () -> dpadDownFrameCount, (c) -> dpadDownFrameCount = c);
        updateButtonFrame(gamepad.dpad_left, () -> dpadLeftFrameCount, (c) -> dpadLeftFrameCount = c);
        updateButtonFrame(gamepad.dpad_right, () -> dpadRightFrameCount, (c) -> dpadRightFrameCount = c);

        updateButtonFrame(gamepad.left_bumper, () -> leftBumperFrameCount, (c) -> leftBumperFrameCount = c);
        updateButtonFrame(gamepad.right_bumper, () -> rightBumperFrameCount, (c) -> rightBumperFrameCount = c);

        updateButtonFrame(gamepad.start, () -> startButtonFrameCount, (c) -> startButtonFrameCount = c);
        updateButtonFrame(gamepad.back, () -> backButtonFrameCount, (c) -> backButtonFrameCount = c);

        // For triggers, consider them "pressed" if they exceed a threshold
        updateButtonFrame(gamepad.left_trigger > 0.2, () -> leftTriggerFrameCount, (c) -> leftTriggerFrameCount = c);
        updateButtonFrame(gamepad.right_trigger > 0.2, () -> rightTriggerFrameCount, (c) -> rightTriggerFrameCount = c);
    }

    private void updateButtonFrame(boolean isPressed, IntSupplier frameCountSupplier, IntConsumer frameCountSetter) {
        if (isPressed) {
            frameCountSetter.accept(frameCountSupplier.getAsInt() + 1);
        } else {
            frameCountSetter.accept(0);
        }
    }

    // First-frame checker methods
    public boolean isFirstA() { return aFrameCount == 1; }
    public boolean isFirstB() { return bFrameCount == 1; }
    public boolean isFirstX() { return xFrameCount == 1; }
    public boolean isFirstY() { return yFrameCount == 1; }

    public boolean isFirstDpadUp() { return dpadUpFrameCount == 1; }
    public boolean isFirstDpadDown() { return dpadDownFrameCount == 1; }
    public boolean isFirstDpadLeft() { return dpadLeftFrameCount == 1; }
    public boolean isFirstDpadRight() { return dpadRightFrameCount == 1; }

    public boolean isFirstLeftBumper() { return leftBumperFrameCount == 1; }
    public boolean isFirstRightBumper() { return rightBumperFrameCount == 1; }
    public boolean isFirstLeftTrigger() { return leftTriggerFrameCount == 1; }
    public boolean isFirstRightTrigger() { return rightTriggerFrameCount == 1; }
    public boolean isFirstStart() {
        return startButtonFrameCount == 1;
    }
    public boolean isFirstBack() {
        return backButtonFrameCount == 1;
    }

    // regular checkers
    // regular checkers
    public boolean a() {
        return gamepad.a;
    }
    public boolean b() {
        return gamepad.b;
    }
    public boolean x() {
        return gamepad.x;
    }
    public boolean y() {
        return gamepad.y;
    }
    public boolean dpadLeft() {
        return gamepad.dpad_left;
    }
    public boolean dpadRight() {
        return gamepad.dpad_left;
    }
    public boolean dpadUp() {
        return gamepad.dpad_up;
    }
    public boolean dpadDown() {
        return gamepad.dpad_down;
    }
    public boolean back() {
        return gamepad.back;
    }
    public boolean start() {
        return gamepad.start;
    }
    public boolean leftBumper() {
        return gamepad.left_bumper;
    }
    public boolean rightBumper() {
        return gamepad.right_bumper;
    }
    public double leftStickX() {
        return gamepad.left_stick_x;
    }
    public double leftStickY() {
        return gamepad.left_stick_y;
    }
    public double rightStickX() {
        return gamepad.right_stick_x;
    }
    public double rightStickY() {
        return gamepad.right_stick_y;
    }
    public double leftTrigger() {
        return gamepad.left_trigger;
    }
    public double rightTrigger() {
        return gamepad.right_trigger;
    }
}