// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // m/s; used to be 3 meters per second.
  public static final double kMaxAngularSpeed = Math.PI / 2.0; // rotations/second; used to be pi = 1/2 rotation per
                                                               // second

  // Swerve Drive Modules 1-4
  // positive X is in front. positive y is to the left.
  private final Translation2d m_backRightLocation = new Translation2d(Inches.of(14.0).times(-1.0),
      Inches.of(12.0).times(-1.0));
  private final Translation2d m_backLeftLocation = new Translation2d(Inches.of(14.0).times(-1.0), Inches.of(12.0));
  private final Translation2d m_frontLeftLocation = new Translation2d(Inches.of(14.0), Inches.of(12.0));
  private final Translation2d m_frontRightLocation = new Translation2d(Inches.of(14.0), Inches.of(12.0).times(-1.0));

  // Swerve Drive Modules 1-4
  private final SwerveModule m_backRight = new SwerveModule(1, 10, 11, 12);
  private final SwerveModule m_backLeft = new SwerveModule(2, 20, 21, 22);
  private final SwerveModule m_frontLeft = new SwerveModule(3, 30, 31, 32);
  private final SwerveModule m_frontRight = new SwerveModule(4, 40, 41, 42);

  // Fancy gyro.
  private final Pigeon2 m_gyro = new Pigeon2(1);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // TODO log more here
    // SmartDashboard.putNumber("gyro rotation degrees:", m_gyro.getRotation2d().getDegrees());
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
    SmartDashboard.putNumber("gyro rotation degrees:", m_gyro.getRotation2d().getDegrees());
  }

  public void resetGyro() {
    m_gyro.reset();
  }
}
