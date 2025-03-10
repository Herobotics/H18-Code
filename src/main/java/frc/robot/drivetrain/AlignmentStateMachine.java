package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AlignmentStateMachine {
    enum FieldAlignment {
        STOPPED,  // hit "soft limit switch" aka angle
        CLEARED   // Free to move again
      }

    FieldAlignment state = FieldAlignment.CLEARED;
    boolean lastAngleAligned = false; // if the angle is aligned

    // Outputs 1 if movement allowed. Otherwise 0.
    public double move(double currentAngle) {
        boolean thisAngleAligned = (Math.abs(currentAngle % Constants.hexAngles) < 2.0); // within 5 past
        if (!lastAngleAligned && thisAngleAligned) {
            // Newly aligned. Stop.
            state = FieldAlignment.STOPPED;
        }
        lastAngleAligned = thisAngleAligned;
        SmartDashboard.putBoolean("Angle alignment?", thisAngleAligned);
        SmartDashboard.putString("Angle alignment state machine", state.toString());
        if (state != FieldAlignment.STOPPED) {
            return 1.0;
        }
        return 0.0;
        
    }

    public void resetState() {
        state = FieldAlignment.CLEARED;
    }
    // https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf
}
