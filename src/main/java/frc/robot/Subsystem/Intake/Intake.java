
package frc.robot.Subsystem.Intake;

import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Intake.IOs.IntakeIO;


public class Intake extends StateControlledSubsystem {
  private static Intake intake;

  private IntakeIO intakeIO = IntakeConstance.getIntakeIO();

  public Intake() {
    super(null, "intake");
  }

  public void setIntakeVoltage() {
    intakeIO.setVoltageIntakeMotor(IntakeConstance.INTAKE_VOLT);
  }

  public void setPositionVoltage() {
    intakeIO.setVoltagePositionMotor(IntakeConstance.POSITION_VOLT);
  }

  public void setPositionToIntake() {
    intakeIO.setIntakePostion(IntakeConstance.IN_INTAKE_POSE);
  }

  public void setPositionToFeeder() {
    intakeIO.setIntakePostion(IntakeConstance.TO_FEEDER_POSE);
  }

  public double getCurrentDrewIntakeMotor() {
    return intakeIO.getCurrentDrawIntakeMotor();
  }

  public double getCurrentDrewPositionMotor() {
    return intakeIO.getCurrentDrawPositionMotor();
  }

  public double getVelocityIntakeMotor() {
    return intakeIO.getVelocityIntakeMotor();
  }

  public double getVelocityPositionMotor() {
    return intakeIO.getVelocityIntakeMotor();
  }

  public double getIntakeMotorTemp() {
    return intakeIO.getIntakeMotorTemp();
  }

  public double getApliedVoltsPositionMotor() {
    return intakeIO.getAplidVoltPositionMotor();
  }

  public double getAplidVoltIntakeMotor() {
    return intakeIO.getAplidVoltIntakeMotor();
  }

  public double getPositionIntakeMotor() {
    return intakeIO.getPositionIntakeMotor();
  }

  public double getIntakePosition() {
    return getIntakePosition();
  }

  public double getPositionMotorTemp() {
    return getPositionMotorTemp();
  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
  }
}
