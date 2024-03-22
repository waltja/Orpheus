// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climbers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.LeftClimber;

public class LeftClimberUp extends Command {
  private LeftClimber climber;
  public LeftClimberUp(LeftClimber climber) {
    this.climber = climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    climber.climberMove(1); // change value
  }
  @Override
  public void end(boolean interrupted) {
    climber.climbersStop();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}