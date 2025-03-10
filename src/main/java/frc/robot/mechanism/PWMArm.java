package frc.robot.mechanism;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class PWMArm {

    VictorSP m_arm = new VictorSP(Constants.ARM_MOTOR_PWM);
    // Voltage for MAX SPEED UP
    private static final Voltage upVoltage = Voltage.ofBaseUnits(3.0, Volts); // just a guess
    // Voltage for down
    private static final Voltage downVoltage = Voltage.ofBaseUnits(3.0, Volts); // gravity's on our side

    public PWMArm() {
        // Do any configuration like setting the motor to be inverted, here.
        m_arm.setInverted(true);
    }

    public void ElevatorMove(double speed) {
        Voltage outputVoltage = Voltage.ofBaseUnits(0.0, Volts);
        if (speed > 0.0) {
            outputVoltage = upVoltage.times(speed);
        } else if (speed < 0) {
            // Ends up negative because speed is negative
            outputVoltage = downVoltage.times(speed);
        }
        SmartDashboard.putNumber("elevator voltage:", outputVoltage.in(Volts));
        this.m_arm.setVoltage(outputVoltage);
    }
}
