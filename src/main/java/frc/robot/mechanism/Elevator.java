package frc.robot.mechanism;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Elevator {

    VictorSP m_elevator = new VictorSP(Constants.ELEVATOR_MOTOR_PWM);
    // Voltage for MAX SPEED UP
    private static final Voltage upVoltage = Voltage.ofBaseUnits(8.0, Volts); // just a guess
    // Voltage for down
    private static final Voltage downVoltage = Voltage.ofBaseUnits(5.0, Volts); // gravity's on our side

    DigitalInput elevatorlimitSwitch = new DigitalInput(2);

    enum ElevatorState {
        MOVING_UP,
        STOPPED,  // stopped by limit switch; must read 0 to resume
        CLEARED,  // cleared to move
        MOVING_DOWN
    }

    ElevatorState state = ElevatorState.CLEARED;
    boolean lastLimitPressed = false;

    public Elevator() {
        // Do any configuration like setting the motor to be inverted, here.
        m_elevator.setInverted(true);
    }

    public void ElevatorMove(double speed) {
        Voltage outputVoltage = Voltage.ofBaseUnits(0.0, Volts);
        boolean thisLimitPressed = !elevatorlimitSwitch.get(); // For this limit switch, true is not pressed
        if (!lastLimitPressed && thisLimitPressed) {
            // Unpressed to pressed. Stop.
            state = ElevatorState.STOPPED;
        }
        if (speed > 0.0) {  // desired: up
            if (state != ElevatorState.STOPPED) {
                outputVoltage = upVoltage.times(speed);
                state = ElevatorState.MOVING_UP;
            }
        } else if (speed < 0.0) {  // desired: down
            if (state != ElevatorState.STOPPED) {
                // Ends up negative because speed is negative
                outputVoltage = downVoltage.times(speed);
                state = ElevatorState.MOVING_DOWN;
            }   
        } else {  // Speed = 0
            state = ElevatorState.CLEARED;
        }

        lastLimitPressed = thisLimitPressed;
        SmartDashboard.putString("elevator state:", state.toString());
        SmartDashboard.putBoolean("elevator limit switch pressed:", thisLimitPressed);
        SmartDashboard.putNumber("elevator voltage:", outputVoltage.in(Volts));
        this.m_elevator.setVoltage(outputVoltage);
    }

    public void LimitReset(){
        state = ElevatorState.CLEARED;
    }
}
