package frc.robot;

public class Constants {
    // As plugged into the RoboRio.
    public static final int ELEVATOR_MOTOR_PWM = 0;
    public static final int ARM_MOTOR_PWM = 1;
    public static final int INTAKE_MOTOR_LEFT_PWM = 2;
    public static final int INTAKE_MOTOR_RIGHT_PWM = 3;

    public static final double PRECISION_MANEUVER_SPEED = 0.4;  // m/s
    public static final double PRECISION_ANGULAR_SPEED = Math.PI / 8.0; // radians/sec

    public static final double ARM_UP_VOLTAGE = 6.0;
    public static final double ARM_DOWN_VOLTAGE = 3.0;

    public static final double hexAngles = 60.0;
}