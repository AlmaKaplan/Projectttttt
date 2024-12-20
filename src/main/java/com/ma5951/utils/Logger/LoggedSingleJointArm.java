// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

/** Add your docs here. */
public class LoggedSingleJointArm {

    private StructArrayPublisher<SwerveModuleState> loggedStates;
    private NetworkTable networkTable;


    public LoggedSingleJointArm(String name) {
        networkTable = NetworkTableInstance.getDefault().getTable("/");
        loggedStates = networkTable.getStructArrayTopic(name , SwerveModuleState.struct).publish();
        loggedStates.set(new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
  });
    }

    public void update(SwerveModuleState[] states) {
        loggedStates.set(states);
    }
}
