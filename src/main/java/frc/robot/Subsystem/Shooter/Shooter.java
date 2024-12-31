
package frc.robot.Subsystem.Shooter;

import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Shooter.IOs.ShooterIO;

public class Shooter extends StateControlledSubsystem {
  public static Shooter shooter;
  public ShooterIO IO = ShooterConstants.get_Shooter_IO();

  public Shooter() {
    super(null,"shooter");
  }

  public void setLeftVoltage() {
    IO.setVoltageLeft(ShooterConstants.VOLTAGE_LEFT);
  }

  public void setVoltageRight() {
    IO.setVoltageRight(ShooterConstants.VOLTAGE_RIGHT);
  }

  public void setSpeedLeft() {
    IO.setSpeedLeft(ShooterConstants.LEFT_SPEED);
  }

  public void setSpeedRight() {
    IO.setSpeedRight(ShooterConstants.RIGHT_SPEED);
  }

  public void setNutralModeRightMotor() {
    IO.setNutralModeRightMotor(true);
  }

  public void setNutralModeLeftMotor() {
    IO.setNutralModeLeftMotor(true);
  }

  public double getRightMotorTemp() {
    return IO.getRightMotorTemp();
  }

  public double getLeftMotorTemp() {
    return IO.getLeftMotorTemp();
  }

  public double getRightMotorApliedVolts() {
    return IO.getRightMotorApliedVolts();
  }

  public double getLeftMotorApliedVolts() {
    return IO.getLeftMotorApliedVolts();
  }

  public double getRightMotorCurrentDraw() {
    return IO.getRightMotorCurrentDraw();
  }

  public double getLeftMotorCurrentDraw() {
    return IO.getLeftMotorCurrentDraw();
  }

  public double getRightMotorVelocity() {
    return IO.getRightMotorVelocity();
  }

  public double getLeftMotorVelocity() {
    return IO.getLeftMotorVelocity();
  }

  public static Shooter getInstace() {
    if (shooter == null) {
      shooter = new Shooter();
    }
    return shooter;
  }

  @Override
  public void periodic() {
    IO.update();
  }
}
