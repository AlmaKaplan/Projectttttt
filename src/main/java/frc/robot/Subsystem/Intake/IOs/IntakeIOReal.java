
package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.PortMap;
import frc.robot.Subsystem.Intake.IntakeConstance;

public class IntakeIOreal implements IntakeIO{

    private TalonFX IntakeMotor;
    private TalonFX positionMotor;
    private TalonFXConfiguration motorConfig;

    public IntakeIOreal() {
        IntakeMotor = new TalonFX(PortMap.IntakePorts.INTAKE_MOTOR_ID);
        positionMotor = new TalonFX(PortMap.IntakePorts.POSTION_MOTOR_ID);
        motorConfig = new TalonFXConfiguration();

        configIntakeMotor();
        configPosition();
    }

    private void configPosition() {
        motorConfig.Feedback.SensorToMechanismRatio = IntakeConstance.INTAKE_GEAR;

        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakReverseVoltage = -12;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstance.IsCurrentLimitEnabled;
        motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstance.PeakCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstance.ContinuesCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = IntakeConstance.PeakCurrentTime;

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = IntakeConstance.KP;
        motorConfig.Slot0.kI = IntakeConstance.KI;
        motorConfig.Slot0.kD = IntakeConstance.KD;

        positionMotor.getConfigurator().apply(motorConfig);
    }


    private void configIntakeMotor() {
        motorConfig.Feedback.SensorToMechanismRatio = IntakeConstance.INTAKE_GEAR;

        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakReverseVoltage = -12;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstance.IsCurrentLimitEnabled;
        motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstance.PeakCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstance.ContinuesCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = IntakeConstance.PeakCurrentTime;

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        IntakeMotor.getConfigurator().apply(motorConfig);
    }

    public void setVoltageIntakeMotor(double volt) {
        IntakeMotor.setVoltage(volt);
    }

    public void setVoltagePositionMotor(double volt) {
        IntakeMotor.setVoltage(volt);
    }

    public void setNutralModeIntakeMotor(boolean isbrake) {
        if (isbrake) {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
    }

    public void setNutralModePositionMotor(boolean isbrake) {
        if (isbrake) {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
    }

    public void setIntakePostion(double position) {
          
    }

    



}
