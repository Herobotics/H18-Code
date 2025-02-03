// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// TODO: https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-frc.html
// Docs: https://api.ctr-electronics.com/phoenix6/release/java/
// TODO: set drive conversion velocity factor and angle conversion factor for each pair of motors in Phoenix Tuner X.
// drive conversion position factor = wheel circumference / drive gear ratio
// drive conversion velocity factor based off that
// angle conversion factor = 360 / angle gear ratio ()
// "The steering gear ratio of the MK4i is 150/7:1"
// N.B. we ordered the L2 drivetrain ratio, which is 6.75:1 not 8.14:1 (watch for this in online code)
// our Colson wheels are 4 inches in diameter

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

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
      // MAYBE: any correction factor needed for velocity or angle conversion due to gearing?
      // MAYBE: use CANcoder for angle?
      m_driveMotor.getVelocity(), new Rotation2d(m_turningMotor.getPosition()));
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
      m_driveMotor.getPosition(), new Rotation2d(m_turningMotor.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_absoluteEncoder.getAbsolutePosition());

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
