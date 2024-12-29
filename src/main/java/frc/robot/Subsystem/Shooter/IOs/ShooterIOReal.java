
package frc.robot.Subsystem.Shooter.IOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.Subsystem.Shooter.ShooterConstants;

public class ShooterIOReal implements ShooterIO {

    protected TalonFX rightMotor;
    protected TalonFXConfiguration rightMotorConfig;
    protected TalonFX leftMotor;
    protected TalonFXConfiguration leftMotorConfig;

    private VelocityVoltage control;

    private StatusSignal<Current> currentDrawRightMotor;
    private StatusSignal<AngularVelocity> velocityRightMotor;
    private StatusSignal<Temperature> rightMotorTemp;
    private StatusSignal<Voltage> appliedVoltageRightMotor;

    private StatusSignal<Current> currentDrawLeftMotor;
    private StatusSignal<AngularVelocity> velocityLeftMotor;
    private StatusSignal<Temperature> leftMotorTemp;
    private StatusSignal<Voltage> appliedVoltageLeftMotor;


    public ShooterIOReal() {
        rightMotor = new TalonFX(PortMap.ShooterPorts.SHOOTER_RIGHT_MOTOR);
        rightMotorConfig = new TalonFXConfiguration();

        leftMotor = new TalonFX(PortMap.ShooterPorts.SHOOTER_LEFT_MOTOR);
        leftMotorConfig = new TalonFXConfiguration();

        control = new VelocityVoltage(ConvUtil.RPStoRPM(ShooterConstants.SPEED));

        currentDrawRightMotor = rightMotor.getSupplyCurrent();
        velocityRightMotor = rightMotor.getVelocity();
        rightMotorTemp = rightMotor.getDeviceTemp();
        appliedVoltageRightMotor = rightMotor.getMotorVoltage();

        currentDrawLeftMotor = leftMotor.getSupplyCurrent();
        velocityLeftMotor = leftMotor.getVelocity();
        leftMotorTemp = leftMotor.getDeviceTemp();
        appliedVoltageLeftMotor = leftMotor.getMotorVoltage();

        rightConfig();
        leftConfig();
    }


    private void rightConfig() {
        rightMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RIGHT;

        rightMotorConfig.Voltage.PeakForwardVoltage = 12;
        rightMotorConfig.Voltage.PeakReverseVoltage = -12;

        rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.RightIsCurrentLimitEnabled;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.RightPeakCurrentTime;
        rightMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.RightContinuesCurrentLimit;
        rightMotorConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.RightPeakCurrentTime;

        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightMotorConfig.Slot0.kP = ShooterConstants.Right_KP;
        rightMotorConfig.Slot0.kI = ShooterConstants.Left_KI;
        rightMotorConfig.Slot0.kD = ShooterConstants.Right_KD;

        rightMotor.getConfigurator().apply(rightMotorConfig);
    }

    private void leftConfig() {
        leftMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_LEFT;

        leftMotorConfig.Voltage.PeakForwardVoltage = 12;
        leftMotorConfig.Voltage.PeakReverseVoltage = -12;

        leftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.LeftIsCurrentLimitEnabled;
        leftMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.LeftPeakCurrentLimit;
        leftMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.LeftContinuesCurrentLimit;
        leftMotorConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.LeftPeakCurrentTime;

        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftMotorConfig.Slot0.kP = ShooterConstants.Left_KP;
        leftMotorConfig.Slot0.kI = ShooterConstants.Left_KI;
        leftMotorConfig.Slot0.kD = ShooterConstants.Left_KD;

        leftMotor.getConfigurator().apply(leftMotorConfig);
    }


    public void setVoltageRight(double volt) {
        rightMotor.setVoltage(volt);
    }

    public void setVoltageLeft(double volt) {
        leftMotor.setVoltage(volt);
    }

    public void setSpeedRight(double speed) {
        rightMotor.setControl(control.withVelocity(speed).withSlot(ShooterConstants.CONTROL_SLOT));
    }

    public void setSpeedLeft(double speed) {
        leftMotor.setControl(control.withVelocity(speed).withSlot(ShooterConstants.CONTROL_SLOT));
    }

    public void setNutralModeRightMotor(boolean isbrake) {
        if (isbrake) {
            rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        rightMotor.getConfigurator().apply(rightMotorConfig);
    }

    public void setNutralModeLeftMotor(boolean isbrake) {
        if (isbrake) {
            leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        leftMotor.getConfigurator().apply(leftMotorConfig);
    }

    public double getRightMotorCurrentDraw() {
        return currentDrawRightMotor.getValueAsDouble();
    }

    public double getLeftMotorCurrentDraw() {
        return currentDrawLeftMotor.getValueAsDouble();
    }

    public double getRightMotorVelocity() {
        return ConvUtil.RPStoRPM(velocityRightMotor.getValueAsDouble());
    }

    public double getLeftMotorVelocity() {
        return ConvUtil.RPStoRPM(velocityLeftMotor.getValueAsDouble());
    }

    public double getRightMotorTemp() {
        return rightMotorTemp.getValueAsDouble();
    }

    public double getLeftMotorTemp() {
        return leftMotorTemp.getValueAsDouble();
    }

    public double getRightMotorApliedVolts() {
        return appliedVoltageRightMotor.getValueAsDouble();
    }

    public double getLeftMotorApliedVolts() {
        return appliedVoltageLeftMotor.getValueAsDouble();
    }

    public void update() {
        BaseStatusSignal.refreshAll(
            currentDrawRightMotor,
            currentDrawLeftMotor,
            velocityRightMotor,
            velocityLeftMotor,
            rightMotorTemp,
            leftMotorTemp,
            appliedVoltageRightMotor,
            appliedVoltageLeftMotor
        );
    }
}
