package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class UnitUtil {
  /**
   * Converts given meters to feet.
   *
   * @param m The meters to convert to feet.
   * @return Feet converted from meters.
   */
  public static double mToft(double m) {
    return Units.metersToFeet(m);
  }

  /**
   * Converts given feet to meters.
   *
   * @param ft The feet to convert to meters.
   * @return Meters converted from feet.
   */
  public static double ftTom(double ft) {
    return Units.feetToMeters(ft);
  }

  /**
   * Converts given meters to inches.
   *
   * @param m The meters to convert to inches.
   * @return Inches converted from meters.
   */
  public static double mToin(double m) {
    return Units.metersToInches(m);
  }

  /**
   * Converts given inches to meters.
   *
   * @param in The inches to convert to meters.
   * @return Meters converted from inches.
   */
  public static double inTom(double in) {
    return Units.inchesToMeters(in);
  }

  /**
   * Converts given degrees to radians.
   *
   * @param deg The degrees to convert to radians.
   * @return Radians converted from degrees.
   */
  public static double degTorad(double deg) {
    return Units.degreesToRadians(deg);
  }

  /**
   * Converts given radians to degrees.
   *
   * @param rad The radians to convert to degrees.
   * @return Degrees converted from radians.
   */
  public static double radTodeg(double rad) {
    return Units.radiansToDegrees(rad);
  }

  /**
   * Converts given radians to rotations.
   *
   * @param rad The radians to convert.
   * @return rotations Converted from radians.
   */
  public static double radTorot(double rad) {
    return Units.radiansToRotations(rad);
  }

  /**
   * Converts given degrees to rotations.
   *
   * @param deg The degrees to convert.
   * @return rotations Converted from degrees.
   */
  public static double degTorot(double deg) {
    return Units.degreesToRotations(deg);
  }

  /**
   * Converts given rotations to degrees.
   *
   * @param rot The rotations to convert.
   * @return degrees Converted from rotations.
   */
  public static double rotTodeg(double rot) {
    return Units.rotationsToDegrees(rot);
  }

  /**
   * Converts given rotations to radians.
   *
   * @param rot The rotations to convert.
   * @return radians Converted from rotations.
   */
  public static double rotTorad(double rot) {
    return Units.rotationsToRadians(rot);
  }

  /**
   * Converts rotations per minute to radians per second.
   *
   * @param RPM The rotations per minute to convert to radians per second.
   * @return Radians per second converted from rotations per minute.
   */
  public static double RPMToradPs(double RPM) {
    return Units.rotationsPerMinuteToRadiansPerSecond(RPM);
  }

  /**
   * Converts radians per second to rotations per minute.
   *
   * @param radPs The radians per second to convert to from rotations per minute.
   * @return Rotations per minute converted from radians per second.
   */
  public static double radPsToRPM(double radPs) {
    return Units.radiansPerSecondToRotationsPerMinute(radPs);
  }

  /**
   * Converts given milliseconds to seconds.
   *
   * @param ms The milliseconds to convert to seconds.
   * @return Seconds converted from milliseconds.
   */
  public static double msTos(double ms) {
    return Units.millisecondsToSeconds(ms);
  }

  /**
   * Converts given seconds to milliseconds.
   *
   * @param s The seconds to convert to milliseconds.
   * @return Milliseconds converted from seconds.
   */
  public static double sToms(double s) {
    return Units.secondsToMilliseconds(s);
  }

  /**
   * Converts kilograms into lbs (pound-mass).
   *
   * @param kg The kilograms to convert to lbs (pound-mass).
   * @return Lbs (pound-mass) converted from kilograms.
   */
  public static double kgTolb(double kg) {
    return Units.kilogramsToLbs(kg);
  }

  /**
   * Converts lbs (pound-mass) into kilograms.
   *
   * @param lb The lbs (pound-mass) to convert to kilograms.
   * @return Kilograms converted from lbs (pound-mass).
   */
  public static double lbTokg(double lb) {
    return Units.lbsToKilograms(lb);
  }
}
