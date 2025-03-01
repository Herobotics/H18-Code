package frc.robot.mechanism;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Constants;

public class Arm {
    PWMVictorSPX m_arm = new PWMVictorSPX(Constants.ARM_MOTOR_PWM);
    // Voltage for MAX SPEED UP
    private static final Voltage upVoltage = Voltage.ofBaseUnits(0.5, Volts); // just a guess
    // Voltage for down
    private static final Voltage downVoltage = Voltage.ofBaseUnits(0.1, Volts); // just a guess, less because gravity's on our side.

    // TODO: Add encoder.

    public Arm() {
        // Do any configuration like setting the motor to be inverted, here.
    }

    public void ArmUp(double speed) {
        this.m_arm.setVoltage(upVoltage.times(speed));
    }

    public void ArmDown() {
        this.m_arm.setVoltage(downVoltage);
    }
}