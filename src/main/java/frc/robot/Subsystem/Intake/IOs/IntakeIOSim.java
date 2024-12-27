
package frc.robot.Subsystem.Intake.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOSim extends IntakeIOreal{

    private TalonFXMotorSim IntakeMotorSim;
    private TalonFXMotorSim PositionMotorSim;

    public IntakeIOSim() {
        super();
        IntakeMotorSim = new TalonFXMotorSim(IntakeMotor, motorConfigIntakeMotor, DCMotor.getFalcon500(1), 0.025);
        PositionMotorSim = new TalonFXMotorSim(PositionMotor ,motorConfigPosition, DCMotor.getKrakenX60(1), 0.2);
    }

    @Override
    public void uptate() {
        super.uptate();
        IntakeMotorSim.updateSim();
        PositionMotorSim.updateSim();
    }
}
