// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRollers;

public class IntakeUp extends Command {
  private IntakePivot intake;
  private IntakeRollers intakeRollers;
  /** Creates a new IntakeUp. */
  public IntakeUp(IntakePivot intake, IntakeRollers intakeRollers) {
    this.intake = intake;
    this.intakeRollers = intakeRollers;
    addRequirements(intake, intakeRollers);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeRollers.stop();;
    intake.retract();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.pivotIsFinished();
  }
}