// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

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
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {
  private static final double kWheelCircumferenceMeters = Units.inchesToMeters(4.0 * Math.PI); // circ = pi * d

  // Swerve Drive Module
  private final int m_moduleNumber; // Saved for printing if necessary
  private final TalonFX m_turningMotor;
  private final TalonFX m_driveMotor;
  private final CANcoder m_absoluteEncoder;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param moduleNumber   Module #, for logging.
   * @param turningMotorID CANbus ID for the turning motor.
   * @param driveMotorID   CANbus ID for the drive motor.
   * @param cancoderID     CANbus ID for the absolute encoder (on top).
   */
  public SwerveModule(
      int moduleNumber,
      int turningMotorID,
      int driveMotorID,
      int cancoderID) {
    m_moduleNumber = moduleNumber;
    m_turningMotor = new TalonFX(turningMotorID);
    m_turningMotor.getConfigurator().apply(new Slot0Configs().withKS(.1).withKP(20));
    m_turningMotor.getConfigurator().apply(new ClosedLoopGeneralConfigs().withContinuousWrap(true));
    m_driveMotor = new TalonFX(driveMotorID);
    m_driveMotor.getConfigurator().apply(new Slot0Configs().withKS(.1).withKV(.7).withKP(.35));

    m_absoluteEncoder = new CANcoder(cancoderID);
    m_absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5));
    if (moduleNumber == 1) {
      m_absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0.148926));
    }
    if (moduleNumber == 2) {
      m_absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0.030273));
    }
    if (moduleNumber == 3) {
      m_absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0.29834));
    }
    if (moduleNumber == 4) {
      m_absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(0.181885));
    }
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        // The turning motor uses the CANcoder for calibration, so they'll read the same
        // value here.
        m_driveMotor.getVelocity().getValueAsDouble() * kWheelCircumferenceMeters,
        new Rotation2d(m_turningMotor.getPosition().getValue()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        // The turning motor uses the CANcoder for calibration, so they'll read the same
        // value here.
        m_driveMotor.getPosition().getValueAsDouble() * kWheelCircumferenceMeters,
        new Rotation2d(m_turningMotor.getPosition().getValue()));
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

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired
    // direction of travel that can occur when modules change directions. This
    // results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Request a velocity from the drive motor.
    final VelocityVoltage m_request_drive = new VelocityVoltage(
        desiredState.speedMetersPerSecond / kWheelCircumferenceMeters);
    m_driveMotor.setControl(m_request_drive);

    // Request a position from the rotation motor.
    // Note getMeasure() returns a typesafe Angle, which doesn't need to be
    // converted.
    final PositionVoltage m_request_turn = new PositionVoltage(desiredState.angle.getMeasure());
    m_turningMotor.setControl(m_request_turn);
  }
}
