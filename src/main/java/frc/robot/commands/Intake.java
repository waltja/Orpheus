// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntake;

public class Intake extends Command {

  private GroundIntake intake;

  public Intake(GroundIntake intake) {

    this.intake = intake;
    addRequirements(intake);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    intake.intake(1, .5);

  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
