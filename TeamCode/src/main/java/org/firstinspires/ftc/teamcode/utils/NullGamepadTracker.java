package org.firstinspires.ftc.teamcode.utils;

public class NullGamepadTracker extends GamepadTracker{

    public NullGamepadTracker() {
        super(null);
    }

    // First-frame checker methods
    public boolean isFirstA() { return false; }
    public boolean isFirstB() { return false; }
    public boolean isFirstX() { return false; }
    public boolean isFirstY() { return false; }

    public boolean isFirstDpadUp() { return false; }
    public boolean isFirstDpadDown() { return false; }
    public boolean isFirstDpadLeft() { return false; }
    public boolean isFirstDpadRight() { return false; }

    public boolean isFirstLeftBumper() { return false; }
    public boolean isFirstRightBumper() { return false; }
    public boolean isFirstLeftTrigger() { return false; }
    public boolean isFirstRightTrigger() { return false; }
    public boolean isFirstStart() {
        return false;
    }
    public boolean isFirstBack() {
        return false;
    }

    // regular checkers
    public boolean a() {
        return false;
    }
    public boolean b() {
        return false;
    }
    public boolean x() {
        return false;
    }
    public boolean y() {
        return false;
    }
    public boolean dpadLeft() {
        return false;
    }
    public boolean dpadRight() {
        return false;
    }
    public boolean dpadUp() {
        return false;
    }
    public boolean dpadDown() {
        return false;
    }
    public boolean back() {
        return false;
    }
    public boolean start() {
        return false;
    }
    public boolean leftBumper() {
        return false;
    }
    public boolean rightBumper() {
        return false;
    }
    public double leftStickX() {
        return 0;
    }
    public double leftStickY() {
        return 0;
    }
    public double rightStickX() {
        return 0;
    }
    public double rightStickY() {
        return 0;
    }
    public double leftTrigger() {
        return 0;
    }
    public double rightTrigger() {
        return 0;
    }
}