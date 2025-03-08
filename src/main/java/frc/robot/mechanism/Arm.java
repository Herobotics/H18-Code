package frc.robot.mechanism;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Arm {
    SparkMax m_arm = new SparkMax(8, MotorType.kBrushed);
    SparkMaxConfig config = new SparkMaxConfig();
    // Voltage for MAX SPEED
    private static final Voltage upVoltage = Voltage.ofBaseUnits(6.0, Volts); // just a guess
    // TODO: Add encoder.

    config.inverted(true).idleMode(IdleMode.kBrake);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    m_arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    public Arm() {
    }

    public void ArmMove(double speed) {
        Voltage outputVoltage = upVoltage.times(speed);
        SmartDashboard.putNumber("arm voltage:", outputVoltage.in(Volts));
        SmartDashboard.putNumber("arm angle:", 0.0);
        this.m_arm.setVoltage(outputVoltage);
    }
}