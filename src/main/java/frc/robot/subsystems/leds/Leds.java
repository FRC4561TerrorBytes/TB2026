package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReasonValue;

public class Leds extends VirtualSubsystem {
  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean endgameAlert = false;
  public boolean autoScoring = false;
  public double autoScoreRotatePercent = 0.0;
  public boolean autoScoreAtRotationSetpoint = false;
  public boolean intakeRunning = false;

  public boolean robotOk = false;
  public boolean driveDisconnected = false;
  public boolean extensionDisconnected = false;
  public boolean indexerDisconnected = false;
  public boolean intakeDisconncted = false;
  public boolean shooterDisconnected = false;
  public boolean visionDisconnected = false;

  public Color hexColor = Color.kDarkGreen;
  public Color secondaryHexColor = Color.kDarkGreen;

  private Optional<Alliance> alliance = Optional.empty();
  private Color disabledColor = Color.kGreen;
  private Color secondaryDisabledColor = Color.kDarkBlue;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private static final boolean prideLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int length = 65;
  private static final Section fullSection = new Section(0, length);
  private static final Section topSection = new Section(length / 2, length);
  private static final Section bottomSection = new Section(0, length / 2);
  private static final Section topQuartSection = new Section((length / 4) * 3, length);
  private static final Section bottomThreeQuartSection = new Section(0, (length / 4) * 3);
  private static final double strobeDuration = 0.2;
  private static final double breathFastDuration = 0.5;
  private static final double breathSlowDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveDisabledCycleLength = 15.0;
  private static final double waveDisabledDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal

  private Leds() {
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    fullSection,
                    Color.kWhite,
                    Color.kBlack,
                    breathSlowDuration,
                    System.currentTimeMillis() / 1000.0);
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      disabledColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(disabledColor);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : secondaryDisabledColor;
    }

    // Update auto state
    if (DriverStation.isEnabled()) {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getTimestamp();
    }

    // Update estop state
    estopped = DriverStation.isEStopped();

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Select LED mode
    
    solid(fullSection, Color.kBlack); // Default to off
    if (estopped) {
      solid(fullSection, Color.kRed);
    } 
    else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto && Timer.getTimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        wave(
            new Section(
                0,
                (int) (length * (1 - ((Timer.getTimestamp() - lastEnabledTime) / autoFadeTime)))),
            Color.kGold,
            Color.kDarkBlue,
            waveFastCycleLength,
            waveFastDuration);
      } else if (prideLeds) {
        // Pride stripes
        stripes(
            fullSection,
            List.of(
                Color.kBlack,
                Color.kRed,
                Color.kOrangeRed,
                Color.kYellow,
                Color.kGreen,
                Color.kBlue,
                Color.kPurple,
                Color.kBlack,
                new Color(0.15, 0.3, 1.0),
                Color.kDeepPink,
                Color.kWhite,
                Color.kDeepPink,
                new Color(0.15, 0.3, 1.0)),
            3,
            5.0);
      } else {
        // Default pattern for disabled
        robotOk = !driveDisconnected && !extensionDisconnected && !indexerDisconnected && !intakeDisconncted && !shooterDisconnected;

        if(Constants.currentMode.equals(Mode.SIM)){
          visionDisconnected = false;
        }
        if(robotOk && !visionDisconnected){
          bounce(disabledColor, Color.kDarkGreen, length/5, 2.0);
        }
        else if(robotOk && visionDisconnected){
          strobe(fullSection, Color.kBlack, Color.kYellow, breathSlowDuration);
        }
        else{
          strobe(fullSection, Color.kBlack, Color.kRed, breathSlowDuration);
        }
      }

    } 
    else if (DriverStation.isAutonomous()) {
      wave(fullSection, Color.kGreen, Color.kPurple, waveFastCycleLength, waveFastDuration);
    } 
    else {
      //Default pattern for teleop
      wave(fullSection, disabledColor, secondaryDisabledColor, waveDisabledCycleLength, waveDisabledDuration);

      // Intake running
      if (intakeRunning) {
        strobe(fullSection, Color.kBlack, Color.kViolet, strobeDuration);
      }

      // Auto scoring
      if (autoScoring) {
        if(autoScoreAtRotationSetpoint){
          //rainbow(fullSection, rainbowCycleLength, rainbowDuration);
          wave(fullSection, Color.kBlack, Color.kYellow, waveFastCycleLength, waveFastDuration);
        }
        else{
          gradient(new Section(0, (int)(autoScoreRotatePercent*length)), disabledColor, Color.kGreen);
          solid(new Section((int)(autoScoreRotatePercent*length), length), Color.kBlack);
        }
      }

      // Endgame alert
      if (endgameAlert) {
        strobe(fullSection, Color.kRed, Color.kGold, strobeDuration);
      }
    }

    // Update LEDs
    leds.setData(buffer);
  }

  /** Sets all LEDs in a section to a solid color 
   * @param section - the section of the LEDs to apply the solid color to
   * @param color - the color to set the section to
  */
  private Color solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        buffer.setLED(i, color);
      }
    }
    return color;
  }

  /**
   * Creates a strobe effect that flashes between two colors in a set duration
   * @param section - the section of the LEDs to apply the strobe to
   * @param c1 - the first color
   * @param c2 - the second color
   * @param duration - the duration of the strobe effect
   */
  private Color strobe(Section section, Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    return solid(section, c1On ? c1 : c2);
  }

  /** Creates a breath effect that fades between two colors in a set duration */
  private Color breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    solid(section, color);
    return color;
  }

  private Color breathCalculate(Section section, Color c1, Color c2, double duration) {
    double x = ((Timer.getTimestamp() % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    return color;
  }
  /** Function to run to create a breath effect */
  private Color breath(Section section, Color c1, Color c2, double duration) {
    return breath(section, c1, c2, duration, Timer.getTimestamp());
  }

  /**
   * Creates a rainbow effect across a section for a set time that moves
   * @param section - the section of the LEDs to apply the rainbow to
   * @param cycleLength - the length of each rainbow cycle
   * @param duration - the duration of the rainbow effect
   */
  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }
  /**
   * Creates a wave effect across a section for a set time that moves
   * @param section - the section of the LEDs to apply the wave to
   * @param c1 - the first color
   * @param c2 - the second color
   * @param cycleLength - the length of each wave cycle
   * @param duration - the duration of the wave effect
  */
  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(length - 1 - i, new Color(red, green, blue));
    }
  }

  /** Creates a stripe effect with a list of colors where the length of each individual stripe is equal 
   * 
   * @param section - the section of the LEDs to apply the stripes to
   * @param colors - the list of colors to use for the stripes
   * @param stripeLength - the length of each individual stripe
   * @param duration - the duration of the stripe effect
   */
  private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
    int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = section.end() - 1; i >= section.start(); i--) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  /**
   * Creates a moving stripe above a background color that bounces from side to side for a specified bounce length at a specified duration
   * @param bgColor - Color that will show up behind the bouncing color
   * @param bounceColor - Color of the bouncing stripe
   * @param bounceLength - Length of the bouncing stripe
   * @param duration - Duration of the bouncing effect
   */
  private void bounce(Color bgColor, Color bounceColor, int bounceLength, double duration){
    int offset = (int) (Timer.getTimestamp() % duration / duration * (length * 2 - 2*bounceLength));
    if(offset > length - bounceLength){
      offset = (length-bounceLength) - (offset - (length-bounceLength));
      
    }
    for(int i = 0; i < length; i++){
      if(i >= offset && i < offset + bounceLength){
        buffer.setLED(i, bounceColor);
      }
      else{
        buffer.setLED(i, bgColor);
      }
    }
  }

  /**
   * Creates a gradient in a section between two colors. 
   * @param section - the section of the LEDs to apply the gradient to
   * @param c1 - the color on the left side of the gradient
   * @param c2 - the color on the right side of the gradient  
   * 
   * @return Gradient on LED strip - sets the LEDs in the section to a gradient between the two colors
   */
  private void gradient(Section section, Color c1, Color c2){
    //HI MANBIR :D
    //int offset = (int)(Timer.getTimestamp() % duration / duration)*length;
    double redDifference = c2.red - c1.red;
    double greenDifference = c2.green - c1.green;
    double blueDifference = c2.blue - c1.blue;

    for(int i = section.start; i < section.end; i++){ 
      double redValue = c1.red + redDifference * i / length;
      double greenValue = c1.green + greenDifference * i / length;
      double blueValue = c1.blue + blueDifference * i / length;

      buffer.setLED(i, new Color(redValue, greenValue, blueValue));  
    }
  }

  private static record Section(int start, int end) {}
}