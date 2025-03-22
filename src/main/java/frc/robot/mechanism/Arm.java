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
import com.revrobotics.spark.SparkBase.ControlType;
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
        config.closedLoop.pidf(3, 0, 0, 0)
                .outputRange(-1, 1); // proportion of max
        m_arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("arm desired setpoint", .1);
        SmartDashboard.putNumber("p:", 3);
        SmartDashboard.putNumber("i:", 0);
        SmartDashboard.putNumber("d:", 0);
        SmartDashboard.putNumber("f:", 0);
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
        ArmSetFeed(SmartDashboard.getNumber("arm desired setpoint", .1));
        // this.m_arm.setVoltage(outputVoltage);
    }

    public void ArmSetFeed(double setpoint) {
this.SetPIDF();
this.m_arm.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
        SmartDashboard.putNumber("arm applied output:", this.m_arm.getAppliedOutput());

    }

    public void SetPIDF(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pidf(SmartDashboard.getNumber("p:", 3), SmartDashboard.getNumber("i:", 0), SmartDashboard.getNumber("d:", 0), SmartDashboard.getNumber("f:", 0));
        
        // Don't persist parameters since it takes time and this change is temporary
        this.m_arm.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}