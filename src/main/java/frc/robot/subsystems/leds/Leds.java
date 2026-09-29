package frc.robot.subsystems.leds;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.VirtualSubsystem;

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
  public boolean passing = false;
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

  // Constants
  private static final boolean prideLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final double autoFadeMaxTime = 5.0; // Return to normal

   /* color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex */
    private static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor kViolet = RGBWColor.fromHSV(3/2 * 3.14, 0.9, 0.8);
    private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();
    private static final RGBWColor kDarkGreen = new RGBWColor(Color.kDarkGreen);
    private static final RGBWColor kGold = new RGBWColor(Color.kGold);
    private static final RGBWColor kYellow = new RGBWColor(Color.kYellow);
    private static final RGBWColor kBlack = new RGBWColor(Color.kBlack);

    private static final int kSlot1StartIdx = 38;
    private static final int kSlot1EndIdx = 67;
  private final CANdle m_candle = new CANdle(1, CANBus.roboRIO());

  private Leds() {
    var cfg = new CANdleConfiguration();
        /* set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(cfg);
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

    // Select LED mode
    
    if (estopped) {
      m_candle.setControl(
        new SolidColor(kSlot1StartIdx, kSlot1EndIdx)
        .withColor(kRed)
      );
    } 
    else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto && Timer.getTimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
         m_candle.setControl(
          new ColorFlowAnimation(kSlot1StartIdx, kSlot1EndIdx)
             .withColor(kGold)
         );
        }
       

      else if (prideLeds) {
        // Pride stripes
       m_candle.setControl(
        new RainbowAnimation(kSlot1StartIdx, kSlot1EndIdx)
       );
        
      } else {
        // Default pattern for disabled
        robotOk = !driveDisconnected && !extensionDisconnected && !indexerDisconnected && !intakeDisconncted && !shooterDisconnected;

        if(Constants.currentMode.equals(Mode.SIM)){
          visionDisconnected = false;
        }
        if(robotOk && !visionDisconnected){
          m_candle.setControl(
          new ColorFlowAnimation(kSlot1StartIdx, kSlot1EndIdx)
             .withColor(kDarkGreen)
         );
        }
        else if(robotOk && visionDisconnected){
          m_candle.setControl(
          new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx)
             .withColor(kYellow)
         );
        }
        else{
          m_candle.setControl(
          new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx)
             .withColor(kRed)
          );
        }
      }

    } 
    else if (DriverStation.isAutonomous()) {
      m_candle.setControl(
          new TwinkleAnimation(kSlot1StartIdx, kSlot1EndIdx)
          .withColor(kDarkGreen)
      );
    } 
    else {
      //Default pattern for teleop
      m_candle.setControl(
          new FireAnimation(kSlot1StartIdx, kSlot1EndIdx)
          );

      // Intake running
      if (intakeRunning) {
        m_candle.setControl(
          new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx)
             .withColor(kViolet)
          );
      }

      //Passing
      if(passing){
        m_candle.setControl(
        new RainbowAnimation(kSlot1StartIdx, kSlot1EndIdx)
       );
      }

      // Auto scoring
      if (autoScoring) {
        if(autoScoreAtRotationSetpoint){
          m_candle.setControl(
          new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx)
             .withColor(kYellow)
          );
        }
        else{
          m_candle.setControl(
          new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx)
             .withColor(kGreen)
          );
          m_candle.setControl(
          new SolidColor(kSlot1StartIdx, kSlot1EndIdx)
             .withColor(kBlack)
          );
        }
      }

      // Endgame alert
      if (endgameAlert) {
        m_candle.setControl(
          new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx)
             .withColor(kGold)
          );
      }
    }
  }
}