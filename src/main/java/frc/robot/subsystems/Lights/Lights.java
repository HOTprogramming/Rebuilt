package frc.robot.subsystems.Lights;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.Lights.LightsConstants.*;

import java.util.ArrayList;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

public class Lights extends SubsystemBase {

    StringPublisher NetworkTableColor;

    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 18;

    private static final int kSlot1StartIdx = 38;
    private static final int kSlot1EndIdx = 67;
    

  private CANBus canBus = new CANBus(LightsConstants.kBusName);
  private final CANdle candle = new CANdle(CANdle_ID, canBus);

  public enum AnimationType {
    None,
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  private AnimationType m_anim0State = AnimationType.None;
  private AnimationType m_anim1State = AnimationType.None;
  private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
  private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();
  private static final RGBWColor KGREEN_COLOR = RGBWColor.fromHex("#00FF00").orElseThrow();

  private AnimationType current = AnimationType.SetAll;

  public Lights() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    /* set the LED strip type and brightness */
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.5;
    /* disable status LED when being controlled */
    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candle.getConfigurator().apply(cfg);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("HOT/Lights");

    NetworkTableColor = table.getStringTopic("COLOR").publish();
  }
  private static Lights instance = null;
  public static Lights getInstance() {
    return instance = instance == null ? new Lights() : instance;
  }

  public void apply(AnimationType anim) {
    current = anim;
  
      switch (anim) {
                default:
                case ColorFlow:
                    candle.setControl(
                        new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case Rainbow:
                    candle.setControl(
                        new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case Twinkle:
                    candle.setControl(
                        new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case TwinkleOff:
                    candle.setControl(
                        new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case Fire:
                    candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
            }
          

        /* if the selection for slot 1 changes, change animations */
            switch (m_anim1State) {
                 case Larson:
                    candle.setControl(
                        new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(kRed)
                    );
                    break;
                 case RgbFade:
                    candle.setControl(
                        new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                    );
                    break;
                 case SingleFade:
                    candle.setControl(
                        new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(kRed)
                    );
                    break;
                 case Strobe:
                    candle.setControl(
                        new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(kRed)
                    );
                    break;
                 case Fire:
                    /* direction can be reversed by either the Direction parameter or switching start and end */
                    candle.setControl(
                        new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withDirection(AnimationDirectionValue.Backward)
                            .withCooling(0.4)
                            .withSparking(0.5)
                    );
                    break;
            }
        }

   public Command setRed() {       
        return runOnce(() ->
        {
            candle.setControl(new SolidColor(0, 18).withColor(kRed)); 
        }); 
  }

  public Command setViolet() {       
        return runOnce(() ->
        {
            candle.setControl(new SolidColor(0, 18).withColor(kViolet)); 

        }); 
  }

   public Command setFire() {       
        return runOnce(() ->
        {
             apply(AnimationType.Fire);

        }); 
    }
    public Command setGreen() {
        return runOnce(() ->
        candle.setControl(new SolidColor(0, 18).withColor(KGREEN_COLOR)));
  }
}            
