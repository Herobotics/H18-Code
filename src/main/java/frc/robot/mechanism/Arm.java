package frc.robot.mechanism;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
    // Voltage for MAX SPEED
    private static final Voltage upVoltage = Voltage.ofBaseUnits(3.0, Volts); // just a guess

    SparkMax m_arm;

    public Arm() {
        m_arm = new SparkMax(8, MotorType.kBrushed); // CANbus 8
        SparkMaxConfig config = new SparkMaxConfig();
        // todo how to add inverted
        config
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        config.encoder.countsPerRevolution(8192); // is this inverted?
        // config.;
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // through bore encoder: 8192 counts per revolution, max rpm 1200
        // quadrature encoder
        config.closedLoop
                .p(0)
                .i(0)
                .d(0)
                .outputRange(0, 3.0);
        // Set MAXMotion parameters
        config.closedLoop.maxMotion
                .maxVelocity(1.0)
                .maxAcceleration(1.0)
                .allowedClosedLoopError(1.0);
        m_arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void ArmMove(double speed) {
        Voltage outputVoltage = upVoltage.times(speed);
        SmartDashboard.putNumber("arm voltage:", outputVoltage.in(Volts));
        SmartDashboard.putNumber("arm angle:", m_arm.getAbsoluteEncoder().getPosition());
        this.m_arm.setVoltage(outputVoltage);
    }

    public void ArmSetFeed() {
        // TODO set setpoint
        // m_controller.setReference(setPoint, ControlType.kPosition);
    }

    public void StopMotor() {
        this.m_arm.stopMotor();
    }
}