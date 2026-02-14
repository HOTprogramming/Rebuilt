package frc.robot.subsystems.Lights;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.Lights.LightsConstants.*;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeConstants;

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

    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 18;

  private CANBus canBus = new CANBus(LightsConstants.kBusName);
  private final CANdle candle = new CANdle(CANdle_ID, canBus);

  public enum AnimationType {
    None,
    ColorFlow,
    Fire1,
    Fire2,
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
  private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
  private static final RGBWColor kBlue = RGBWColor.fromHex("#233AE800").orElseThrow();
  private static final RGBWColor kGreen = RGBWColor.fromHex("#3DCE6500").orElseThrow();
  private static final RGBWColor kYellow = RGBWColor.fromHex("#EFF21800").orElseThrow();
  private static final RGBWColor kOrange = RGBWColor.fromHex("#F2820A00").orElseThrow();
  private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();
  private static final RGBWColor kPink = RGBWColor.fromHex("#F20C8B00").orElseThrow();
  private static final RGBWColor kWhite = RGBWColor.fromHex("#FFFFFF00").orElseThrow();

  private AnimationType current = AnimationType.SetAll;

  public Lights() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    /* set the LED strip type and brightness */
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.5;
    /* disable status LED when being controlled */
    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candle.getConfigurator().apply(cfg);
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
                case Fire1:
                    candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case Larson:
                    candle.setControl(
                        new LarsonAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(1)
                            .withColor(kRed)
                    );
                    break;
                case RgbFade:
                    candle.setControl(
                        new RgbFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(1)
                    );
                    break;
                case SingleFade:
                    candle.setControl(
                        new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(1)
                            .withColor(kRed)
                    );
                    break;
                case Strobe:
                    candle.setControl(
                        new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(1)
                            .withColor(kRed)
                    );
                    break;
                case Fire2:
                    /* direction can be reversed by either the Direction parameter or switching start and end */
                    candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(1)
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

   public Command setOrange() {       
        return runOnce(() ->
        {
            candle.setControl(new SolidColor(0, 18).withColor(kOrange)); 
        }); 
   }

   public Command setYellow() {       
        return runOnce(() ->
        {
            candle.setControl(new SolidColor(0, 18).withColor(kYellow)); 
        }); 
   }

   public Command setGreen() {       
        return runOnce(() ->
        {
            candle.setControl(new SolidColor(0, 18).withColor(kGreen)); 
        }); 
   }

   public Command setBlue() {       
        return runOnce(() ->
        {
            candle.setControl(new SolidColor(0, 18).withColor(kBlue)); 
        }); 
    }

    public Command setViolet() {       
        return runOnce(() ->
        {
            candle.setControl(new SolidColor(0, 18).withColor(kViolet)); 

        }); 
    }

    public Command setFire1() {       
        return runOnce(() ->
        {
             apply(AnimationType.Fire1);

        }); 
    }

    public Command setPink() {       
        return runOnce(() ->
        {
            candle.setControl(new SolidColor(0, 18).withColor(kPink)); 
        }); 
    }

    public Command setWhite() {       
        return runOnce(() ->
        {
            candle.setControl(new SolidColor(0, 18).withColor(kWhite)); 
        }); 
    }

    public Command setRainbow() {
        return runOnce(() ->
        {
            apply(AnimationType.Rainbow);
        });
    }

    public Command setTwinkle() {
        return runOnce(() ->
        {
            apply(AnimationType.Twinkle);
        });
    }
}            