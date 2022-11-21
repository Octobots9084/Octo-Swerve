package org.octobots.robot;

public class MotorIDs {
    // Global Constants
    public static final String CANFD_NAME = "can1";

    // Drive Train
    public static final int FRONT_RIGHT_DRIVE = 1;
    public static final int FRONT_RIGHT_STEER = 2;
    public static final int FRONT_LEFT_DRIVE = 3;
    public static final int FRONT_LEFT_STEER = 4;
    public static final int BACK_LEFT_DRIVE = 5;
    public static final int BACK_LEFT_STEER = 6;
    public static final int BACK_RIGHT_DRIVE = 7;
    public static final int BACK_RIGHT_STEER = 8;

    // Climb
    public static final int SOLENOID_LOW_OPEN = 4;
    public static final int SOLENOID_LOW_CLOSED = 7;
    public static final int SOLENOID_MID_OPEN = 2;
    public static final int SOLENOID_MID_CLOSED = 5;
    public static final int SOLENOID_HIGH_OPEN = 3;
    public static final int SOLENOID_HIGH_CLOSED = 6;
    public static final int CLIMB_ROTATE_A = 10;
    public static final int CLIMB_ROTATE_B = 11;
    public static final int LOW_SWITCH_A = 1;
    public static final int LOW_SWITCH_B = 2;
    public static final int MID_SWITCH_A = 3;
    public static final int MID_SWITCH_B = 4;
    public static final int HIGH_SWITCH_A = 5;
    public static final int HIGH_SWITCH_B = 6;
    public static final int CLIMB_ENCODER = 7;

    // Collect
    public static final int COLLECT_INTAKE = 13;
    public static final int COLLECT_BELTS = 12;
    public static final int COLLECT_WHEEL = 14;

    // Shooter
    public static final int SHOOTER_LEFT = 15;
    public static final int SHOOTER_ANGLE = 16;
    public static final int SHOOTER_RIGHT = 17;

    private MotorIDs() {
    }
}
