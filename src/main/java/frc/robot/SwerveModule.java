// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Docs: https://api.ctr-electronics.com/phoenix6/release/java/
// Done?: set drive conversion velocity factor and angle conversion factor for each pair of motors in Phoenix Tuner X.
// drive conversion position factor = wheel circumference / drive gear ratio
// drive conversion velocity factor based off that
// angle conversion factor = 360 / angle gear ratio ()
// "The steering gear ratio of the MK4i is 150/7:1" = 21.4286
// N.B. we ordered the L2 drivetrain ratio, which is 6.75:1 not 8.14:1 (watch for this in online code)
// our Colson wheels are 4 inches in diameter

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  // Swerve Drive Module
  private final int m_moduleNumber;
  private final TalonFX m_turningMotor;
  private final TalonFX m_driveMotor;
  private final CANcoder m_absoluteEncoder;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param moduleNumber Module #, for logging.
   * @param turningMotorID CANbus ID for the turning motor.
   * @param driveMotorID CANbus ID for the drive motor.
   * @param cancoderID CANbus ID for the absolute encoder (on top).
   */
  public SwerveModule(
      int moduleNumber,
      int turningMotorID,
      int driveMotorID,
      int cancoderID) {
    m_moduleNumber = moduleNumber;
    m_turningMotor = new TalonFX(turningMotorID);
    m_driveMotor = new TalonFX(driveMotorID);
    m_absoluteEncoder = new CANcoder(cancoderID);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // TODO: can we do this in Phoenix Tuner X?
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      // MAYBE: use CANcoder for angle?
      // TODO: THIS VELOCITY IS WRONG. This is an Angular Velocity, but the state expects a LinearVelocity. 
      m_driveMotor.getVelocity().getValueAsDouble(), new Rotation2d(m_turningMotor.getPosition().getValue()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      // MAYBE: any correction factor needed for velocity or angle conversion due to gearing?
      // MAYBE: use CANcoder for angle?
      // TODO: THIS POSITION IS WRONG. This position is an angle, and we want a distance.
      m_driveMotor.getPosition().getValueAsDouble(), new Rotation2d(m_turningMotor.getPosition().getValue()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_absoluteEncoder.getAbsolutePosition().getValue());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Request a velocity from the drive motor.
    // TODO: desired state is given in meters per second. we want velocityVoltage to be in rotations per second.
    // See https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#converting-from-meters
    final MotionMagicVelocityVoltage m_request_drive = new MotionMagicVelocityVoltage(desiredState.speedMetersPerSecond);
    m_driveMotor.setControl(m_request_drive);

    // Request a position from the rotation motor.
    // N.B. getMeasure() returns a typesafe Angle, which doesn't need to be converted.
    final MotionMagicVoltage m_request_turn = new MotionMagicVoltage(desiredState.angle.getMeasure());
    m_turningMotor.setControl(m_request_turn);
  }
}
