package frc.robot.mechanism;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Arm {
    PWMVictorSPX m_arm = new PWMVictorSPX(Constants.ARM_MOTOR_PWM);
    // Voltage for MAX SPEED
    private static final Voltage upVoltage = Voltage.ofBaseUnits(6.0, Volts); // just a guess
    // TODO: Add encoder.
    

    public Arm() {
        m_arm.setInverted(true);
    }

    public void ArmMove(double speed) {
        Voltage outputVoltage = upVoltage.times(speed);
        SmartDashboard.putNumber("arm voltage:", outputVoltage.in(Volts));
        this.m_arm.setVoltage(outputVoltage);
    }
}