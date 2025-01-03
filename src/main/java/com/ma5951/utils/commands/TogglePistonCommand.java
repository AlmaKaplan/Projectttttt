// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import com.ma5951.utils.Subsystems.PistonSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class TogglePistonCommand extends Command {
    private PistonSubsystem subsystem;

    public TogglePistonCommand(PistonSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (subsystem.isOpen()) {
            subsystem.close();
        } else {
            subsystem.open();
        }
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return true;
    }
}
