package org.pwrup.napoleon.trajectory;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryBuilder {

  private final List<Waypoint> trajectoryWaypoints;

  private static PathConstraints constraints;
  private static GoalEndState endState;
  private static IdealStartingState startingState;

  public TrajectoryBuilder(double[] transformations) {
    this.trajectoryWaypoints = new ArrayList<>();
    for (int i = 0; i < transformations.length; i += 6) {
      Pose2d pose2d = new Pose2d(
          transformations[i],
          transformations[i + 1],
          new Rotation2d(
              new Matrix<>(
                  Nat.N2(),
                  Nat.N2(),
                  new double[] {
                      transformations[i + 2],
                      transformations[i + 3],
                      transformations[i + 4],
                      transformations[i + 5],
                  })));
      this.trajectoryWaypoints.add(new Waypoint(pose2d.getTranslation(), null, null)); // TODO: fix the nulls because idk what they are...
    }

    PathPlannerPath path = new PathPlannerPath(
        trajectoryWaypoints, constraints, startingState, endState);
  }

  public double[] getPositionAt(double timeSinceStart) {
    return null;
  }

  public static void setConstraints(PathConstraints pathConstraints) {
    TrajectoryBuilder.constraints = pathConstraints;
  }

  public static void setEndState(GoalEndState endState) {
    TrajectoryBuilder.endState = endState;
  }

  public static void setStartingState(IdealStartingState startingState) {
    TrajectoryBuilder.startingState = startingState;
  }
}
