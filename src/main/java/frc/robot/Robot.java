// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.mechanism.Elevator;
import frc.robot.mechanism.Arm;
import frc.robot.mechanism.Intake;

public class Robot extends TimedRobot {
  private final XboxController m_driver_controller = new XboxController(0);
  private final XboxController m_operator_controller = new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final Intake claw = new Intake();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    boolean fieldRelative = true;
    if(m_driver_controller.getRightTriggerAxis() > 0.4) {
      fieldRelative = false;
    }
    driveWithJoystick(fieldRelative);
    operatorControls();
  }

  private void operatorControls() {
    // Button for intake out
    // if(m_operator_controller.getBButton()){
    if(m_operator_controller.getRawButton(2)) {
      claw.setIntakemotor(-1.0);
    // Button for intake in
    } else if(m_operator_controller.getRawButton(3)) {
    // } else if(m_operator_controller.getAButton()){
      claw.setIntakemotor(1.0);
    } else {
      claw.setIntakemotor(0.0);
    }

    // arm.ArmMove(MathUtil.applyDeadband(m_operator_controller.getRawAxis(1), 0.1));
    // arm.ArmMove(MathUtil.applyDeadband(m_operator_controller.getLeftY(), 0.1)); // up and down. negative should be down.
    // Use the dpad for the arm.
    // The dpad is a POV controller.
    int dpadDirection = m_operator_controller.getPOV();
    if(dpadDirection == 0) {  // up
      arm.ArmMove(MathUtil.applyDeadband(1.0, 0.1));
    }
    else if(dpadDirection == 180) {  // down
      arm.ArmMove(MathUtil.applyDeadband(-1.0, 0.1));
    }

    elevator.ElevatorMove(MathUtil.applyDeadband(m_operator_controller.getRawAxis(3), 0.1)); // up and down. negative should be down.
    // elevator.ElevatorMove(MathUtil.applyDeadband(m_operator_controller.getRightY(), 0.1)); // up and down. negative should be down.
  }

  private void driveWithJoystick(boolean fieldRelative) {
    if(m_driver_controller.getStartButtonPressed()){
      m_swerve.resetGyro();
    }

    // The dpad is a POV controller.
    int dpadDirection = m_driver_controller.getPOV();
    if(dpadDirection == 0) {  // up, forwards
      m_swerve.drive(Constants.PRECISION_MANEUVER_SPEED, 0, 0, false, getPeriod());
    }
    else if(dpadDirection == 180) {  // down, back
      m_swerve.drive(Constants.PRECISION_MANEUVER_SPEED * -1.0, 0, 0, false, getPeriod());
    }
    else if(dpadDirection == 90) {  // right, + y
      m_swerve.drive(0, -1.0 * Constants.PRECISION_MANEUVER_SPEED, 0, false, getPeriod());
    }
    else if(dpadDirection == 270) {  // left, - y
      m_swerve.drive(0, Constants.PRECISION_MANEUVER_SPEED, 0, false, getPeriod());
    } else {

      double effectiveMaxSpeed = Drivetrain.kMaxSpeed;
      if(m_driver_controller.getLeftBumperButton()){
        effectiveMaxSpeed = Drivetrain.kMaxSpeed * .25;  // 25% NOTE TEST THIS
      }
      
      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      final var xSpeed =
          -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driver_controller.getLeftY(), 0.2))
              * effectiveMaxSpeed;

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      final var ySpeed =
          -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driver_controller.getLeftX(), 0.2))
              * effectiveMaxSpeed;

      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positive in
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      final var rot =
          m_rotLimiter.calculate(MathUtil.applyDeadband(m_driver_controller.getRightX(), 0.4))
              * Drivetrain.kMaxAngularSpeed;

      // TODO: lower/limit acceleration
      m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
    }
  }
}
