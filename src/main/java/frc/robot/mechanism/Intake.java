package frc.robot.mechanism;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;

// public class mechanism extends Mechanism
public class Intake {
    private PWMVictorSPX left_intake_motor = new PWMVictorSPX(Constants.INTAKE_MOTOR_LEFT_PWM);
    private PWMVictorSPX right_intake_motor = new PWMVictorSPX(Constants.INTAKE_MOTOR_RIGHT_PWM);

    private Voltage intakeVoltage = Voltage.ofBaseUnits(6.0, Volts); // just a guess

    public Intake() {
        // Do any configuration like setting a motor to be inverted, here.
        left_intake_motor.setInverted(true);
        right_intake_motor.setInverted(false);
    }

    public void setIntakemotor(double speed) {
        Voltage outputVoltage = intakeVoltage.times(speed);

        SmartDashboard.putNumber("intake voltage", outputVoltage.in(Volts));
        this.left_intake_motor.setVoltage(outputVoltage);
        this.right_intake_motor.setVoltage(outputVoltage);
    }

}