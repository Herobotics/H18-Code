package frc.robot.Mechanism;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class mechanism extends Mechanism

private VictorSP intakemotor;

//Change () to PWM port number
public intakemotor (1){
intakemotor = new VictorSP(0)

}

    VictorSP leftFrontVictorSP = null;
    VictorSP leftBackTalonVictorSP = null;
    VictorSP rightFrontTalonVictorSP = null;
    VictorSP rightBackTalonVictorSP = null;
// intake window motors



public VictorSP getIntakemotor() {
    return intakemotor;
}

public void setIntakemotor(VictorSP intakemotor) {
    this.intakemotor = intakemotor;
}