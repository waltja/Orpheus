// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollers;

public class Intake extends Command {

  private IntakeRollers intake;

  public Intake(IntakeRollers intake) {

    this.intake = intake;
    addRequirements(intake);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    intake.intake(-.7);

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