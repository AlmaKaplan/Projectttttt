
package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    protected TalonFX IntakeMotor;
    protected TalonFX PositionMotor;
    protected TalonFXConfiguration motorConfigIntakeMotor;
    protected TalonFXConfiguration motorConfigPosition;
    

    private PositionVoltage PositionControl;

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
        PositionMotor = new TalonFX(PortMap.IntakePorts.POSTION_MOTOR_ID);
        motorConfigIntakeMotor = new TalonFXConfiguration();
        motorConfigPosition = new TalonFXConfiguration();

        currentDrawIntakeMotor = IntakeMotor.getSupplyCurrent();
        velocityIntakeMotor =  IntakeMotor.getVelocity();
        intakeMotorTemp = IntakeMotor.getDeviceTemp();
        appliedVoltageIntakeMotor = IntakeMotor.getMotorVoltage();
        positionIntakeMotor = IntakeMotor.getPosition();

        currentDrawPositionMotor = PositionMotor.getSupplyCurrent();
        velocityPositionMotor =  PositionMotor.getVelocity();
        positionMotorTemp = PositionMotor.getDeviceTemp();
        appliedVoltagePositionMotor = PositionMotor.getMotorVoltage();
        positionPositionMotor = PositionMotor.getPosition();

        configIntakeMotor();
        configPosition();
    }

    private void configPosition() {
        motorConfigPosition.Feedback.SensorToMechanismRatio = IntakeConstance.INTAKE_GEAR;

        motorConfigPosition.Voltage.PeakForwardVoltage = 12;
        motorConfigPosition.Voltage.PeakReverseVoltage = -12;

        motorConfigPosition.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstance.IsCurrentLimitEnabled;
        motorConfigPosition.CurrentLimits.SupplyCurrentLimit = IntakeConstance.PeakCurrentLimit;
        motorConfigPosition.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstance.ContinuesCurrentLimit;
        motorConfigPosition.CurrentLimits.SupplyCurrentLowerTime = IntakeConstance.PeakCurrentTime;

        motorConfigPosition.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigPosition.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfigPosition.Slot0.kP = IntakeConstance.KP;
        motorConfigPosition.Slot0.kI = IntakeConstance.KI;
        motorConfigPosition.Slot0.kD = IntakeConstance.KD;

        PositionMotor.getConfigurator().apply(motorConfigPosition);
    }


    private void configIntakeMotor() {
        motorConfigIntakeMotor.Feedback.SensorToMechanismRatio = IntakeConstance.INTAKE_GEAR;

        motorConfigIntakeMotor.Voltage.PeakForwardVoltage = 12;
        motorConfigIntakeMotor.Voltage.PeakReverseVoltage = -12;

        motorConfigIntakeMotor.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstance.IsCurrentLimitEnabled;
        motorConfigIntakeMotor.CurrentLimits.SupplyCurrentLimit = IntakeConstance.PeakCurrentLimit;
        motorConfigIntakeMotor.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstance.ContinuesCurrentLimit;
        motorConfigIntakeMotor.CurrentLimits.SupplyCurrentLowerTime = IntakeConstance.PeakCurrentTime;

        motorConfigIntakeMotor.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigIntakeMotor.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        IntakeMotor.getConfigurator().apply(motorConfigIntakeMotor);
    }

    public void setVoltageIntakeMotor(double volt) {
        IntakeMotor.setVoltage(volt);
    }

    public void setVoltagePositionMotor(double volt) {
        IntakeMotor.setVoltage(volt);
    }

    public void setNutralModeIntakeMotor(boolean isbrake) {
        if (isbrake) {
            motorConfigIntakeMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfigIntakeMotor.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
    }

    public void setNutralModePositionMotor(boolean isbrake) {
        if (isbrake) {
            motorConfigPosition.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfigPosition.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
    }

    public void setIntakePostion(double position) {
        PositionMotor.setControl(PositionControl.withPosition(position).withSlot(IntakeConstance.CONTROL_SLOT));
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
