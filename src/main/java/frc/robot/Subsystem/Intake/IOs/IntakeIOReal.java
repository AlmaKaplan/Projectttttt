
package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.Subsystem.Intake.IntakeConstance;

public class IntakeIOreal implements IntakeIO{

    private TalonFX IntakeMotor;
    private TalonFX positionMotor;
    private TalonFXConfiguration motorConfig;

    private StatusSignal<Current> currentDrawIntakeMotor;
    private StatusSignal<AngularVelocity> velocityIntakeMotor;
    private StatusSignal<Temperature> intakeMotorTemp;
    private StatusSignal<Voltage> appliedVoltageIntakeMotor;
    private StatusSignal<Angle> positionIntakeMotor;

    private StatusSignal<Current> currentDrawPositionMotor;
    private StatusSignal<AngularVelocity> velocityPositionMotor;
    private StatusSignal<Temperature> positionMotorTemp;
    private StatusSignal<Voltage> appliedVoltagePositionMotor;
    private StatusSignal<Angle> positionPositionMotor;

    public IntakeIOreal() {
        IntakeMotor = new TalonFX(PortMap.IntakePorts.INTAKE_MOTOR_ID);
        positionMotor = new TalonFX(PortMap.IntakePorts.POSTION_MOTOR_ID);
        motorConfig = new TalonFXConfiguration();

        currentDrawIntakeMotor = IntakeMotor.getSupplyCurrent();
        velocityIntakeMotor =  IntakeMotor.getVelocity();
        intakeMotorTemp = IntakeMotor.getDeviceTemp();
        appliedVoltageIntakeMotor = IntakeMotor.getMotorVoltage();
        positionIntakeMotor = IntakeMotor.getPosition();

        currentDrawPositionMotor = positionMotor.getSupplyCurrent();
        velocityPositionMotor =  positionMotor.getVelocity();
        positionMotorTemp = positionMotor.getDeviceTemp();
        appliedVoltagePositionMotor = positionMotor.getMotorVoltage();
        positionPositionMotor = positionMotor.getPosition();

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

    public double getCurrentDrawIntakeMotor() {
        currentDrawIntakeMotor.refresh();
        return currentDrawIntakeMotor.getValueAsDouble();
    }

    public double getVelocityIntakeMotor() {
        velocityIntakeMotor.refresh();
        return velocityIntakeMotor.getValueAsDouble();
    }

    public double getIntakeMotorTemp() {
        intakeMotorTemp.refresh();
        return intakeMotorTemp.getValueAsDouble();
    }

    public double getAplidVoltIntakeMotor() {
        appliedVoltageIntakeMotor.refresh();
        return appliedVoltageIntakeMotor.getValueAsDouble();
    }

    public double getPositionIntakeMotor() {
        positionIntakeMotor.refresh();
        return positionIntakeMotor.getValueAsDouble();
    }

    public double getCurrentDrawPositionMotor() {
        currentDrawPositionMotor.refresh();
        return currentDrawPositionMotor.getValueAsDouble();
    }

    public double getVelocityPositionMotor() {
        velocityPositionMotor.refresh();
        return velocityPositionMotor.getValueAsDouble();
    }

    public double getPositionMotorTemp() {
        positionMotorTemp.refresh();
        return positionMotorTemp.getValueAsDouble();
    }

    public double getAplidVoltPositionMotor() {
        appliedVoltagePositionMotor.refresh();
        return appliedVoltagePositionMotor.getValueAsDouble();
    }

    public double getPositionPositionMotor() {
        positionPositionMotor.refresh();
        return positionPositionMotor.getValueAsDouble();
    }

    public void uptate() {
        getCurrentDrawIntakeMotor();
        getVelocityIntakeMotor();
        getIntakeMotorTemp();
        getAplidVoltIntakeMotor();
        getPositionIntakeMotor();
        getCurrentDrawPositionMotor();
        getVelocityPositionMotor();
        getPositionMotorTemp();
        getAplidVoltPositionMotor();
        getPositionPositionMotor();
    }
}
