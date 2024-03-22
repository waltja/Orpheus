// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

public class ManualPivotIntake extends Command {
  private IntakePivot intake;
  private DoubleSupplier ySup;
  /** Creates a new ManualPivotIntake. */
  public ManualPivotIntake(IntakePivot intake, DoubleSupplier ySup) {
    this.intake = intake;
    this.ySup = ySup;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yVal = MathUtil.applyDeadband(ySup.getAsDouble(), Constants.stickDeadband);
    intake.manualRotate(yVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}