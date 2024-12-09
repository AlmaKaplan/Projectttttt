
package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.Subsystem.Intake.IntakeConstance;

public class IntakeIOReal implements IntakeIO {

    private TalonFX motor;
    private TalonFXConfiguration motorCunfig;

    private StatusSignal<Current> currentDraw;
    private StatusSignal<AngularVelocity> velocity;
    private StatusSignal<Temperature> motorTemp;
    private StatusSignal<Voltage> getAppliedVolt;

    public IntakeIOReal() {
        motor = new TalonFX(PortMap.IntakePorts.MOTOR_ID);
        motorCunfig = new TalonFXConfiguration();

        currentDraw = motor.getStatorCurrent();
        velocity = motor.getVelocity();
        motorTemp = motor.getDeviceTemp();
        getAppliedVolt = motor.getMotorVoltage();


        Config();
    }

    private void Config() {
        motorCunfig.Feedback.RotorToSensorRatio = IntakeConstance.INTAKE_GEAR;

        motorCunfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        motorCunfig.Voltage.PeakForwardVoltage = 12;
        motorCunfig.Voltage.PeakReverseVoltage = -12;

        motorCunfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstance.IsCurrentLimitEnabled;
        motorCunfig.CurrentLimits.SupplyCurrentLimit = IntakeConstance.PeakCurrentLimit;
        motorCunfig.CurrentLimits.SupplyCurrentLowerTime = IntakeConstance.PeakCurrentTime;
        motorCunfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstance.ContinuesCurrentLimit;

        motor.getConfigurator().apply(motorCunfig);
    }


    public double getCurrentDraw() {
        return currentDraw.getValueAsDouble();
    }

    public double getVelocity() {
        return ConvUtil.RPStoRPM(getVelocity());
    }

    public double getMotorTemp() {
        return motorTemp.getValueAsDouble();
    }

    public double getAppliedVolts() {
    return getAppliedVolt.getValueAsDouble();
    }

    public void setNutralMode(boolean isBrake) {
        if (isBrake) {
            motorCunfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorCunfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }

        motor.getConfigurator().apply(motorCunfig);
    }


    public void setVoltage(double volt) {
        motor.setVoltage(volt);
    }

    public void updatePeriodic() {

        BaseStatusSignal.refreshAll(
            currentDraw,
            velocity,
            motorTemp,
            getAppliedVolt
        );
    }


}
