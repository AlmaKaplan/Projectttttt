
package frc.robot.Subsystem.Shooter.IOs;

public interface ShooterIO {

    void setVoltageRight(double volt);

    void setVoltageLeft(double volt);

    void setNutralModeRightMotor(boolean isBrake);

    void setNutralModeLeftMotor(boolean isBrake);

    void setSpeedLeft(double speed);

    void setSpeedRight(double speed);

    double getRightMotorTemp();

    double getLeftMotorTemp();

    double getRightMotorApliedVolts();

    double getLeftMotorApliedVolts();

    double getRightMotorCurrentDraw();

    double getLeftMotorCurrentDraw();

    double getRightMotorVelocity();

    double getLeftMotorVelocity();

    void update();
} 
