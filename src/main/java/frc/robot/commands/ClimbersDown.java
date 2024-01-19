package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;

public class ClimbersDown extends Command {
  private Climbers climbers;
  public ClimbersDown(Climbers climbers) {
    this.climbers = climbers;
    addRequirements(climbers);
  }
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    climbers.climberMove(-1);
  }
  @Override
  public void end(boolean interrupted) {
    climbers.climbersStop();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}
