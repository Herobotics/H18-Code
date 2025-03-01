package frc.robot.mechanism;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Elevator {

    PWMVictorSPX m_elevator = new PWMVictorSPX(Constants.ELEVATOR_MOTOR_PWM);
    // Voltage for MAX SPEED UP
    private static final Voltage upVoltage = Voltage.ofBaseUnits(3.0, Volts); // just a guess
    // Voltage for down
    private static final Voltage downVoltage = Voltage.ofBaseUnits(-1.0, Volts); // just a guess, less because gravity's on our side.

    public Elevator() {
        // Do any configuration like setting the motor to be inverted, here.
    }

    public void ElevatorUp(double speed) {
        Voltage outputVoltage = upVoltage.times(speed);
        SmartDashboard.putNumber("elevator voltage:", outputVoltage.in(Volts));
        this.m_elevator.setVoltage(outputVoltage);
    }

    public void ElevatorDown() {
        this.m_elevator.setVoltage(downVoltage);
    }
}
