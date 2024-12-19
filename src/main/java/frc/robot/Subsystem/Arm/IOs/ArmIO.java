
package frc.robot.Subsystem.Arm.IOs;

public interface ArmIO {
    void  setNutralMode(boolean isBrake);
    void resetPosition(double newPose);
    double getVelocity();
}
