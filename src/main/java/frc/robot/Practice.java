// package frc.robot;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// public class Robot extends TimedRobot {
//   private final PWMSparkMax leftMotor1 = new PWMSparkMax(1);
//   private final PWMSparkMax leftMotor2 = new PWMSparkMax(1);
//   private final PWMSparkMax rightMotor1 = new PWMSparkMax(1);
//   private final PWMSparkMax rightMotor2 = new PWMSparkMax(1);
  
//   private final DifferentialDrive drive = new DifferentialDrive(leftMotor1, rightMotor1);
//   private final Joystick joystick = new Joystick(0);

//   @Override
//   public void robotInit() {
//     leftMotor2.follow(leftMotor1);
//     rightMotor2.follow(rightMotor1);
//   }

//   @Override
//   public void teleopPeriodic() {
//     double leftSpeed = joystick.getRawAxis(1); // Left stick Y-axis
//     double rightSpeed = joystick.getRawAxis(5); // Right stick Y-axis
//     drive.tankDrive(leftSpeed, rightSpeed);
//   }
// }