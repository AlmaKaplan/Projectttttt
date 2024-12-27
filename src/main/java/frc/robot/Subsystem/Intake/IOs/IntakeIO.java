
package frc.robot.Subsystem.Intake.IOs;

public interface IntakeIO {

    void setVoltageIntakeMotor(double volt);

    void setVoltagePositionMotor(double volt);

    void setNutralModeIntakeMotor(boolean isBrake);

    void setNutralModePositionMotor(boolean isBrake);

    void setIntakePostion(double position);

    double getCurrentDrawIntakeMotor();

    double getCurrentDrawPositionMotor();

    double getAplidVoltIntakeMotor();

    double getAplidVoltPositionMotor();

    double getIntakePosition();

    void uptate();

}
