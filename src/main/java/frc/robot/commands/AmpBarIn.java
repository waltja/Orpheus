// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Shooter;

public class AmpBarIn extends Command {
  private AmpBar ampBar;
  private Shooter shooter;
  /** Creates a new AmpBarOut. */
  public AmpBarIn(AmpBar ampBar, Shooter shooter) {
    this.ampBar = ampBar;
    addRequirements(ampBar);

    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampBar.retract();
    shooter.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ampBar.isFinished();
  }
}
