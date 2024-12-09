
package frc.robot.Subsystem.Intake;

import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.Subsystem.Intake.IOs.IntakeIOReal;
import frc.robot.Subsystem.Intake.IOs.IntakeIOSim;

public class IntakeConstance {
    public static final double INTAKE_GEAR = 1;
    public static final double PeakCurrentLimit = 30; 
    public static final double ContinuesCurrentLimit = 25; 
    public static final double PeakCurrentTime = 0.1; 
    public static final boolean IsCurrentLimitEnabled = true; 



    public static final IntakeIO getIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeIOReal();
        } else {
            return new IntakeIOSim();
        }
    }
}
