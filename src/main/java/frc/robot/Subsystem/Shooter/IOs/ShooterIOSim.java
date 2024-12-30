
package frc.robot.Subsystem.Shooter.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterIOSim extends ShooterIOReal {
    private TalonFXMotorSim rightMotorSim;
    private TalonFXMotorSim leftMotorSim;

    public ShooterIOSim() {
        super();
        rightMotorSim = new TalonFXMotorSim(rightMotor, rightMotorConfig, DCMotor.getFalcon500(1), 0.02);
        rightMotorSim = new TalonFXMotorSim(leftMotor, leftMotorConfig, DCMotor.getFalcon500(1), 0.025);
    }

    @Override
    public void update() {
        super.update();
        rightMotorSim.updateSim();
        leftMotorSim.updateSim();
    }
}
