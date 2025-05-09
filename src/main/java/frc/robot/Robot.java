// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drivetrain.AlignmentStateMachine;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.mechanism.Elevator;
import frc.robot.mechanism.Arm;
import frc.robot.mechanism.PWMArm;
import frc.robot.mechanism.Intake;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  private final XboxController m_driver_controller = new XboxController(0);
  private final XboxController m_operator_controller = new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final Intake claw = new Intake();
  private final AlignmentStateMachine alignment = new AlignmentStateMachine();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  // TODO: lower/limit acceleration
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  
  private long autonomous_start_time;

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void autonomousInit() {
    // Step 1: Start facing drivers station
    m_swerve.resetGyro(180.0);  // We start with the robot facing towards the drivers station.
    autonomous_start_time = System.currentTimeMillis();  // set this to zero
  }

  @Override
   
  public void autonomousPeriodic() {
    // 1000 = 1 second
    m_swerve.updateOdometry();
  
    // New Auto (Auto 2):
    final long duration_milliseconds = System.currentTimeMillis() - autonomous_start_time;

    // Step 2: Drive 2.2 meters forwards (relative to the robot, still backwards relative to DS)
    // time * auto speed = distance, so get time by dividing
    final double drive_forward_time = Constants.AUTO_DISTANCE * 1000 / Constants.AUTO_SPEED; // seconds
    if (duration_milliseconds < drive_forward_time) {   // First few seconds: drive forwards
      // TODO: make sure this actually drives far enough
      claw.setIntakemotor(0);
      arm.ArmSetAuto();
      m_swerve.drive(-1.0 * Constants.AUTO_SPEED, 0, 0, true, getPeriod());
    } else if (duration_milliseconds < (drive_forward_time + 1000)) {  // 1s angle
      // Step 3: Set angle for unloading.
      m_swerve.drive(0, 0, 0, true, getPeriod());
      arm.ArmSetAuto();
    } else if (duration_milliseconds < (drive_forward_time + 1000 + 3000)) {   // 3s unload onto reef
      claw.setIntakemotor(1.0);
    } else if (duration_milliseconds < (drive_forward_time + 1000 + 3000 + 3000)) {   // 3s
      // Move sideways
      m_swerve.drive(0, 1.0 * Constants.AUTO_SPEED, 0, true, getPeriod());
    } else {  // Step 5: Need to back up -- can't be touching the piece.
      // Don't need a distance/time, can set the A-stop.
      m_swerve.drive(1.0 * Constants.AUTO_SPEED, 0, 0, true, getPeriod());
    }
  }

  @Override
  public void teleopPeriodic() {
    boolean fieldRelative = true; 
    if (m_driver_controller.getRightTriggerAxis() > 0.4) {
      fieldRelative = false;
    }
    driveWithJoystick(fieldRelative);
    controlIntake();
    armControls();
    elevatorControl();
  }

  // Intake controls. B pulls coral in, A pushes coral out.
  private void controlIntake() {
    double intakeSpeed = 0.0;  // -1 to 1, % of max intake speed
    // Button for intake out
    if(m_operator_controller.getBButton()){
      intakeSpeed = -1.0;
    // Button for intake in
    } else if(m_operator_controller.getAButton()){
      intakeSpeed = 1.0;
    }
    SmartDashboard.putNumber("intake speed (1 in, -1 out): ", intakeSpeed);
    claw.setIntakemotor(intakeSpeed);
  }

  // Dpad moves arm up and down.
  // Arm can also be moved by left joystick on operator console.
  private void armControls() {
    double armMovement = 0.0; // up and down. negative should be down.

    // The dpad is a POV controller.
    int dpadDirection = m_operator_controller.getPOV();
    if (m_operator_controller.getYButton()) {
      arm.ArmSetScoring();
    } else if (m_operator_controller.getXButton()) {
      arm.ArmSetIntake();
    } else {
      if (dpadDirection == 180) { // down
        armMovement = -1.0;
      } else if (dpadDirection == 0) { // up
        armMovement = 1.0;
      } else {
        armMovement = MathUtil.applyDeadband(-1.0 * m_operator_controller.getLeftY(), 0.1);
      }
      SmartDashboard.putNumber("arm movement (1 up, -1 down): ", armMovement);
      arm.ArmMove(armMovement);
    } 
  }

  // Elevator movement. Right joystick.
  // Forward = up, back = down.
  // Will stop at the limit switch until you lay off the joystick. Then will move again.
  // If it gets stuck, reset state with back/start.
  private void elevatorControl() {
    double elevatorMovement = MathUtil.applyDeadband(-1.0 * m_operator_controller.getRightY(),
    0.1);
    SmartDashboard.putNumber("elevator movement (1 up, -1 down): ", elevatorMovement);
    elevator.ElevatorMove(elevatorMovement); // up and down. negative should be down.

    if (m_operator_controller.getBackButtonPressed() || m_operator_controller.getStartButtonPressed()) {
      elevator.LimitReset();
    }
  }

  // Start and Back reset the gyro.
  // Dpad = precision adjustment
  private void driveWithJoystick(boolean fieldRelative) {
    // Make sure you're facing the robot the same direction as the drivers station. Intake = front.
    if (m_driver_controller.getStartButtonPressed() || m_driver_controller.getBackButtonPressed()) {
      m_swerve.resetGyro(0.0);
    }

    // The dpad is a POV controller.
    // TODO why is it jerky when dpad is released?
    int dpadDirection = m_driver_controller.getPOV();
    if (dpadDirection == 0) { // up, forwards
      m_swerve.drive(m_xspeedLimiter.calculate(Constants.PRECISION_MANEUVER_SPEED), m_yspeedLimiter.calculate(0.0), 0, false, getPeriod());
    } else if (dpadDirection == 180) { // down, back
      m_swerve.drive(m_xspeedLimiter.calculate(Constants.PRECISION_MANEUVER_SPEED * -1.0), m_yspeedLimiter.calculate(0.0), 0, false, getPeriod());
    } else if (dpadDirection == 90) { // right, - y
      m_swerve.drive(m_xspeedLimiter.calculate(0.0), m_yspeedLimiter.calculate(-1.0 * Constants.PRECISION_MANEUVER_SPEED), 0, false, getPeriod());
    } else if (dpadDirection == 270) { // left, + y
      m_swerve.drive(m_xspeedLimiter.calculate(0.0), m_yspeedLimiter.calculate(Constants.PRECISION_MANEUVER_SPEED), 0, false, getPeriod());
    } else if (m_driver_controller.getXButton()){
      double output = alignment.move(m_swerve.getGyro()) * Constants.PRECISION_ANGULAR_SPEED;
      m_swerve.drive(m_xspeedLimiter.calculate(0.0), m_yspeedLimiter.calculate(0), m_rotLimiter.calculate(output), false, getPeriod());
    } else if (m_driver_controller.getBButton()) {
      double output = alignment.move(m_swerve.getGyro()) * -1.0 * Constants.PRECISION_ANGULAR_SPEED;
      m_swerve.drive(m_xspeedLimiter.calculate(0.0), m_yspeedLimiter.calculate(0), m_rotLimiter.calculate(output), false, getPeriod());
    }
    else {
      alignment.resetState();

      double effectiveMaxSpeed = Drivetrain.kMaxSpeed;
      if (m_driver_controller.getLeftBumperButton()) {
        effectiveMaxSpeed = Drivetrain.kMaxSpeed * .25; // 25% NOTE TEST THIS
      }

      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driver_controller.getLeftY(), 0.2))
          * effectiveMaxSpeed;

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driver_controller.getLeftX(), 0.2))
          * effectiveMaxSpeed;

      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positive in
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      final var rot = m_rotLimiter.calculate(MathUtil.applyDeadband(m_driver_controller.getRightX(), 0.2))
          * Drivetrain.kMaxAngularSpeed;

      m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
    }
  }
}
