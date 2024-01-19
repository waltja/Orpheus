package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers;
public class ClimbersUp extends Command {
  private Climbers climbers;
  public ClimbersUp(Climbers climbers) {
    this.climbers = climbers;
    addRequirements(climbers);
  }
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    climbers.climberMove(1); // change value
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
