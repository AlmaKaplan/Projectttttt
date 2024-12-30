
package frc.robot.Subsystem.Shooter;

import frc.robot.Robot;
import frc.robot.Subsystem.Shooter.IOs.ShooterIO;
import frc.robot.Subsystem.Shooter.IOs.ShooterIOReal;
import frc.robot.Subsystem.Shooter.IOs.ShooterIOSim;

public class ShooterConstants {

    public static final double SHOOTER_GEAR_LEFT = 3;

    public static final double LeftPeakCurrentLimit = 40; 
    public static final double LeftContinuesCurrentLimit = 30; 
    public static final double LeftPeakCurrentTime = 0.1; 
    public static final boolean LeftIsCurrentLimitEnabled = true;

    public static final double Left_KP = 0.1;
    public static final double Left_KI = 0;
    public static final double Left_KD = 0;

    public static final double SHOOTER_GEAR_RIGHT = 3;

    public static final double RighttPeakCurrentLimit = 40; 
    public static final double RightContinuesCurrentLimit = 30; 
    public static final double RightPeakCurrentTime = 0.1; 
    public static final boolean RightIsCurrentLimitEnabled = true;

    public static final double Right_KP = 0.1;
    public static final double Right_KI = 0;
    public static final double Right_KD = 0;

    public static final int CONTROL_SLOT = 0;

    public static final double RIGHT_SPEED = 8000;
    public static final double LEFT_SPEED = 6000;
    public static final double VOLTAGE_LEFT = 6;
    public static final double VOLTAGE_RIGHT = 8;
    

    public static final ShooterIO get_Shooter_IO() {
        if (Robot.isReal()) {
            return new ShooterIOReal();
        }
        return new ShooterIOSim();
    }
}
