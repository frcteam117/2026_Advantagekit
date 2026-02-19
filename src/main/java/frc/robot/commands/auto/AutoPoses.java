package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class AutoPoses { // WILL THIS EVEN WORK IDFK
  // ALL THESE POSES ARE AFTER LINE UP AND 8 FUEL SHOOT
  //
  public static final Pose2d RED_LEFT_TRENCH = new Pose2d(13.305, 0.6, new Rotation2d());
  public static final Pose2d RED_LEFT_BUMP = new Pose2d(13.305, 2.5, new Rotation2d());
  public static final Pose2d RED_RIGHT_TRENCH = new Pose2d(13.305, 7.389, new Rotation2d());
  public static final Pose2d RED_RIGHT_BUMP = new Pose2d(13.305, 5.525, new Rotation2d());
  // public static final Pose2d RED_LEFT_TRENCH = (new Pose2d(13.305,0.6, new Rotation2d()));
  // public static final Pose2d RED_LEFT_TRENCH = (new Pose2d(13.305,0.6, new Rotation2d()));
  public static final Pose2d NEUTRAL_CENTER = new Pose2d(8.27, 4, new Rotation2d());

  //
  public static final List<Pose2d> AUTO1_POSE2DS = Arrays.asList();
  // ===
  public static final Optional<Alliance> alliance = DriverStation.getAlliance();
  public static final String allianceColor =
      alliance.get().toString(); // check what this returns to see if its "Red"/"Blue"

  public AutoPoses(String leftOrRight, String trenchOrBump) {
    // String leftOrRightEnd,
    // String trenchOrBumpEnd) { // this assumes all autos will behave the same
    // - with the same differentiation (IDK IF THATS TRUE)
    // put a lot of these poses in a constants file bc they may be useful outside of auto too????
    // USE PATHPLANNER FOR ALL OF THIS!!! and it auto flips it for alliance sides, so thats a lot of
    // stuff that
    // -  you don't have to do
    int startGoalTagID = 0;
    int offsetSign = 0;
    double fromHubOffsetX = 2.5; // ADD THESE IN A CONSTANTS FILE TOO???
    double fromHubOffsetY = 0.0; // ADD THESE IN A CONSTANTS FILE TOO???

    if (allianceColor == "Red") { // if on red side, add x,y,rot
      startGoalTagID = 9; // or 10
      offsetSign = 1;
    } else if (allianceColor == "Blue") { // if on blue side, subtract x,y,rot
      startGoalTagID = 25; // or 26
      offsetSign = -1;
    } //

    /// ADD FOR EACH POSE LIST=====================
    AUTO1_POSE2DS.add(new Pose2d(
        RobotContainer.AprilTagPoses.get(startGoalTagID).getX()
            + fromHubOffsetX * offsetSign, // go to a tag
        RobotContainer.AprilTagPoses.get(startGoalTagID).getY()
            + fromHubOffsetY * offsetSign, // ADD OFFSETS FROM THIS!!!! maybe set their
        // - -/+ sign when setting startGoalTagID
        Rotation2d.fromDegrees(0) // does this need to be the difference of smth? idk
        ));

    // ==============================================
    // add error catcher for if no alliance?
    if (allianceColor == "Red") { // add all other auto poses in here as well?
      if (leftOrRight == "left") { // go to more decimal points?
        if (trenchOrBump == "trench") { // red AZ, using left trench at
          AUTO1_POSE2DS.add(RED_LEFT_TRENCH);
        } else if (trenchOrBump == "bump") { // red AZ, using left bump at
          AUTO1_POSE2DS.add(RED_LEFT_BUMP); // TODO: CONT. ADDING POSES
        }
      } else if (leftOrRight == "right") {
        if (trenchOrBump == "trench") { // red AZ, using right trench
          AUTO1_POSE2DS.add(RED_RIGHT_TRENCH);
        } else if (trenchOrBump == "bump") { // red AZ, using right bump
          AUTO1_POSE2DS.add(RED_RIGHT_BUMP);
        }
      }
    } else if (allianceColor == "Blue") {
      if (leftOrRight == "left") { //
        if (trenchOrBump == "trench") {

        } else if (trenchOrBump == "bump") {

        }
      } else if (leftOrRight == "right") {
        if (trenchOrBump == "trench") {

        } else if (trenchOrBump == "bump") {

        }
      }
    }
    AUTO1_POSE2DS.add(new Pose2d(
        AUTO1_POSE2DS.get(AUTO1_POSE2DS.size() - 1).getX() + 5.035 * offsetSign,
        AUTO1_POSE2DS.get(AUTO1_POSE2DS.size() - 1).getY(),
        new Rotation2d()));
    // targetPoses.add(newpose);
    AUTO1_POSE2DS.add(NEUTRAL_CENTER);
    // ` 11111111

  }
}
