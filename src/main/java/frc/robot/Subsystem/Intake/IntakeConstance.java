
package frc.robot.Subsystem.Intake;

import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.Subsystem.Intake.IOs.IntakeIOSim;
import frc.robot.Subsystem.Intake.IOs.IntakeIOreal;

public class IntakeConstance {
    public static final double INTAKE_GEAR =1;
    public static final double POSITION_GEAR =1;
    public static final double INTAKE_PeakCurrentLimit = 40; 
    public static final double INTAKE_ContinuesCurrentLimit = 30; 
    public static final double INTAKE_PeakCurrentTime = 0.1; 
    public static final boolean INTAKE_IsCurrentLimitEnabled = true;
    public static final double POSITION_PeakCurrentLimit = 40; 
    public static final double POSITION_ContinuesCurrentLimit = 30; 
    public static final double POSITION_PeakCurrentTime = 0.1; 
    public static final boolean POSITION_IsCurrentLimitEnabled = true;

    public static final double KP_POSITION = 0.1;
    public static final double KI_POSITION = 0;
    public static final double KD_POSITION = 0;

    public static final int CONTROL_SLOT = 0;

    public static final double INTAKE_VOLT = 8;
    public static final double POSITION_VOLT = 8;
    public static final double IN_INTAKE_POSE = 0;
    public static final double TO_FEEDER_POSE = 45.535655434234;


    public static final IntakeIO getIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeIOreal();
        } 
        return new IntakeIOSim();
    }
    
}
