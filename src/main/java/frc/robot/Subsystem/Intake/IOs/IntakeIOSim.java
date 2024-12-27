
package frc.robot.Subsystem.Intake.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOSim extends IntakeIOreal{

    private TalonFXMotorSim intakeMotorSim;
    private TalonFXMotorSim positionMotorSim;

    public IntakeIOSim() {
        super();
        intakeMotorSim = new TalonFXMotorSim(intakeMotor, motorConfigIntakeMotor, DCMotor.getFalcon500(1), 0.025);
        positionMotorSim = new TalonFXMotorSim(positionMotor ,motorConfigPosition, DCMotor.getKrakenX60(1), 0.2);
    }

    @Override
    public void uptate() {
        super.uptate();
        intakeMotorSim.updateSim();
        positionMotorSim.updateSim();
    }
}
