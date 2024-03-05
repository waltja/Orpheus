package frc.robot.commands.Climbers;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RightClimber;
public class RightClimberUp extends Command {
  private RightClimber climber;
  public RightClimberUp(RightClimber climber) {
    this.climber = climber;
    addRequirements(climber);
  }
  @Override
  public void initialize() {}
  @Override
  public void execute() {
    climber.climberMove(-1); // change value
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