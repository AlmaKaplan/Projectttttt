
package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.BaseStatusSignal;
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

    protected TalonFX intakeMotor;
    protected TalonFX positionMotor;
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
        intakeMotor = new TalonFX(PortMap.IntakePorts.INTAKE_MOTOR_ID);
        positionMotor = new TalonFX(PortMap.IntakePorts.POSTION_MOTOR_ID);
        motorConfigIntakeMotor = new TalonFXConfiguration();
        motorConfigPosition = new TalonFXConfiguration();

        PositionControl = new PositionVoltage(0);

        currentDrawIntakeMotor = intakeMotor.getSupplyCurrent();
        velocityIntakeMotor =  intakeMotor.getVelocity();
        intakeMotorTemp = intakeMotor.getDeviceTemp();
        appliedVoltageIntakeMotor = intakeMotor.getMotorVoltage();
        positionIntakeMotor = intakeMotor.getPosition();

        currentDrawPositionMotor = positionMotor.getSupplyCurrent();
        velocityPositionMotor =  positionMotor.getVelocity();
        positionMotorTemp = positionMotor.getDeviceTemp();
        appliedVoltagePositionMotor = positionMotor.getMotorVoltage();
        positionPositionMotor = positionMotor.getPosition();

        configIntakeMotor();
        configPosition();
    }

    private void configPosition() {
        motorConfigPosition.Feedback.SensorToMechanismRatio = IntakeConstance.INTAKE_GEAR;

        motorConfigPosition.Voltage.PeakForwardVoltage = 12;
        motorConfigPosition.Voltage.PeakReverseVoltage = -12;

        motorConfigPosition.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstance.POSITION_IsCurrentLimitEnabled;
        motorConfigPosition.CurrentLimits.SupplyCurrentLimit = IntakeConstance.POSITION_PeakCurrentLimit;
        motorConfigPosition.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstance.POSITION_ContinuesCurrentLimit;
        motorConfigPosition.CurrentLimits.SupplyCurrentLowerTime = IntakeConstance.POSITION_PeakCurrentTime;

        motorConfigPosition.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigPosition.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfigPosition.Slot0.kP = IntakeConstance.KP_POSITION;
        motorConfigPosition.Slot0.kI = IntakeConstance.KI_POSITION;
        motorConfigPosition.Slot0.kD = IntakeConstance.KD_POSITION;

        positionMotor.getConfigurator().apply(motorConfigPosition);
    }


    private void configIntakeMotor() {
        motorConfigIntakeMotor.Feedback.SensorToMechanismRatio = IntakeConstance.INTAKE_GEAR;

        motorConfigIntakeMotor.Voltage.PeakForwardVoltage = 12;
        motorConfigIntakeMotor.Voltage.PeakReverseVoltage = -12;

        motorConfigIntakeMotor.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstance.INTAKE_IsCurrentLimitEnabled;
        motorConfigIntakeMotor.CurrentLimits.SupplyCurrentLimit = IntakeConstance.INTAKE_PeakCurrentLimit;
        motorConfigIntakeMotor.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstance.INTAKE_ContinuesCurrentLimit;
        motorConfigIntakeMotor.CurrentLimits.SupplyCurrentLowerTime = IntakeConstance.INTAKE_PeakCurrentTime;

        motorConfigIntakeMotor.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigIntakeMotor.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeMotor.getConfigurator().apply(motorConfigIntakeMotor);
    }

    public void setVoltageIntakeMotor(double volt) {
        intakeMotor.setVoltage(volt);
    }

    public void setVoltagePositionMotor(double volt) {
        positionMotor.setVoltage(volt);
    }

    public void setNutralModeIntakeMotor(boolean isbrake) {
        if (isbrake) {
            motorConfigIntakeMotor.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfigIntakeMotor.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        intakeMotor.getConfigurator().apply(motorConfigIntakeMotor);
    }

    public void setNutralModePositionMotor(boolean isbrake) {
        if (isbrake) {
            motorConfigPosition.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfigPosition.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        positionMotor.getConfigurator().apply(motorConfigPosition);
    }

    public void setIntakePostion(double position) {
        positionMotor.setControl(PositionControl.withPosition(position).withSlot(IntakeConstance.CONTROL_SLOT));
    }

    public double getCurrentDrawIntakeMotor() {
        return currentDrawIntakeMotor.getValueAsDouble();
    }

    public double getVelocityIntakeMotor() {
        return velocityIntakeMotor.getValueAsDouble();
    }

    public double getIntakeMotorTemp() {
        return intakeMotorTemp.getValueAsDouble();
    }

    public double getAplidVoltIntakeMotor() {
        return appliedVoltageIntakeMotor.getValueAsDouble();
    }

    public double getPositionIntakeMotor() {
        return positionIntakeMotor.getValueAsDouble();
    }

    public double getCurrentDrawPositionMotor() {
        return currentDrawPositionMotor.getValueAsDouble();
    }

    public double getVelocityPositionMotor() {
        return velocityPositionMotor.getValueAsDouble();
    }

    public double getPositionMotorTemp() {
        return positionMotorTemp.getValueAsDouble();
    }

    public double getAplidVoltPositionMotor() {
        return appliedVoltagePositionMotor.getValueAsDouble();
    }

    public double getIntakePosition() {
        return positionPositionMotor.getValueAsDouble();
    }

    public void uptate() {
        BaseStatusSignal.refreshAll(
            currentDrawIntakeMotor,
            velocityIntakeMotor,
            intakeMotorTemp,
            appliedVoltageIntakeMotor,
            positionIntakeMotor,
            currentDrawPositionMotor,
            velocityPositionMotor,
            positionMotorTemp,
            appliedVoltagePositionMotor,
            positionPositionMotor

        );
    }
}
