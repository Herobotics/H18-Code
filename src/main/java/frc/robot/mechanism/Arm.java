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
import frc.robot.Constants;

public class Arm {
    // Voltage for MAX SPEED
    private static final Voltage upVoltage = Voltage.ofBaseUnits(Constants.ARM_UP_VOLTAGE, Volts); // just a guess
    private static final Voltage downVoltage = Voltage.ofBaseUnits(Constants.ARM_DOWN_VOLTAGE, Volts); // just a guess

    SparkMax m_arm;

    public Arm() {
        m_arm = new SparkMax(7, MotorType.kBrushed); // CANbus 7

        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        config.absoluteEncoder.zeroOffset(0.439);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // through bore encoder: 8192 counts per revolution, max rpm 1200
        // quadrature encoder
        config.closedLoop.pidf(.3, 0, 0, .1)
                .outputRange(-.5, .5); // proportion of max
        m_arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void ArmMove(double speed) {
        Voltage outputVoltage = Voltage.ofBaseUnits(0.0, Volts);
        if(speed > 0.0){
            outputVoltage = upVoltage.times(speed);
        } else if (speed < 0.0) {
            outputVoltage = downVoltage.times(speed);
        } 
        SmartDashboard.putNumber("arm voltage:", outputVoltage.in(Volts));
        SmartDashboard.putNumber("arm angle:", m_arm.getAbsoluteEncoder().getPosition());
        this.m_arm.setVoltage(outputVoltage);
    }

    public void ArmSetFeed(double setpoint) {
        this.m_arm.setReference(setpoint, ControlType.kPosition);
    }
}