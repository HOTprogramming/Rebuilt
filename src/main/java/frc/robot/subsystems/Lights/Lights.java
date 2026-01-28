package frc.robot.subsystems.Lights;

import static frc.robot.subsystems.Lights.LightsConstants.*;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.controls.Off;

import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.LEDStripType;
import com.ctre.phoenix6.signals.VBatOutputMode;

public class Lights extends SubsystemBase {

  private final CANdle candle = new CANdle(CANdle_ID, "rio");

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    AutoAlign,
    SetAll
  }

  private AnimationTypes current = AnimationTypes.SetAll;

  public Lights() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.statusLedOffWhenActive = false;
    cfg.disableWhenLOS = false;
    cfg.stripType = LEDStripType.RGB;
    cfg.brightnessScalar = 1.0;
    cfg.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(cfg);

    apply(AnimationTypes.SetAll);
  }

  private void apply(AnimationTypes anim) {
    current = anim;

    final int ledStart = Start;
    final int ledEnd = Start + LEDS_PER_ANIMATION - 1; // inclusive

    switch (anim) {

      case ColorFlow ->
        candle.setControl(
          new ColorFlowAnimation(ledStart, ledEnd)
            .withColor(new RGBWColor(128, 20, 70))
            .withSpeed(0.7)
            .withDirection(ColorFlowAnimation.Direction.Forward)
        );

      case Fire ->
        candle.setControl(
          new FireAnimation(ledStart, ledEnd)
            .withSpeed(0.7)
            .withBrightness(0.7)
        );

      case Larson ->
        candle.setControl(
          new LarsonAnimation(ledStart, ledEnd)
            .withColor(new RGBWColor(0, 255, 46))
            .withSpeed(1.0)
            .withBounceMode(LarsonAnimation.BounceMode.Front)
            .withSize(3)
        );

      case Rainbow ->
        candle.setControl(
          new RainbowAnimation(ledStart, ledEnd)
            .withSpeed(1.0)
            .withBrightness(0.8)
        );

      case RgbFade ->
        candle.setControl(
          new RgbFadeAnimation(ledStart, ledEnd)
            .withSpeed(0.7)
            .withBrightness(0.4)
        );

      case SingleFade ->
        candle.setControl(
          new SingleFadeAnimation(ledStart, ledEnd)
            .withColor(new RGBWColor(50, 2, 200))
            .withSpeed(0.5)
        );

      case Strobe ->
        candle.setControl(
          new StrobeAnimation(ledStart, ledEnd)
            .withColor(new RGBWColor(240, 10, 180))
            .withSpeed(0.5)
        );

      case Twinkle ->
        candle.setControl(
          new TwinkleAnimation(ledStart, ledEnd)
            .withColor(new RGBWColor(30, 70, 60))
            .withSpeed(0.4)
            .withTwinklePercent(TwinkleAnimation.TwinklePercent.Percent6)
        );

      case TwinkleOff ->
        candle.setControl(
          new TwinkleOffAnimation(ledStart, ledEnd)
            .withColor(new RGBWColor(70, 90, 175))
            .withSpeed(0.8)
            .withTwinkleOffPercent(TwinkleOffAnimation.TwinkleOffPercent.Percent100)
        );

      case AutoAlign ->
        candle.setControl(
          new StrobeAnimation(ledStart, ledEnd)
            .withColor(new RGBWColor(0, 255, 0))
            .withSpeed(0.3)
        );

      case SetAll ->
        candle.setControl(new Off());
    }
  }


  public Command setAnimation(AnimationTypes anim) {
    return runOnce(() -> apply(anim));
  }

  public Command stopAnimation() {
    return runOnce(() -> candle.setControl(new Off()));
  }

  // Solid colors (uses your Start + count)
  public Command setGreen()  { return runOnce(() -> candle.setLEDs(0,   255, 0,   0, Start, count)); }
  public Command setRed()    { return runOnce(() -> candle.setLEDs(255, 0,   0,   0, Start, count)); }
  public Command setYellow() { return runOnce(() -> candle.setLEDs(225, 150, 0,   0, Start, count)); }
  public Command setPurple() { return runOnce(() -> candle.setLEDs(255, 0,   255, 0, Start, count)); }
  public Command setWhite()  { return runOnce(() -> candle.setLEDs(255, 255, 255, 0, Start, count)); }
  public Command setTeal()   { return runOnce(() -> candle.setLEDs(0,   128, 128, 0, Start, count)); }

  // One LED indicator (good = teal-ish, bad = red-ish)
  public Command setOne(int ledNum, boolean good) {
    return runOnce(() -> candle.setLEDs(good ? 0 : 255, good ? 200 : 0, 0, 0, ledNum, 1))
      .ignoringDisable(true);
  }

  // Your camera segment example (unchanged)
  public void leftCameraLights(int r, int g, int b) {
    candle.setLEDs(r, g, b, 0, 8, 10);
  }

  // Blink an animation on/off for repeats
  public Command blink(AnimationTypes anim, int repeats, double timeSeconds) {
    Command one =
      Commands.sequence(
        setAnimation(anim),
        Commands.waitSeconds(timeSeconds),
        setAnimation(AnimationTypes.SetAll),
        Commands.waitSeconds(timeSeconds)
      );

    ArrayList<Command> seq = new ArrayList<>();
    for (int i = 0; i < Math.max(0, repeats); i++) seq.add(one);

    return seq.isEmpty() ? Commands.none() : Commands.sequence(seq.toArray(new Command[0]));
  }

  // Cycle animations
  public void incrementAnimation() {
    switch (current) {
      case ColorFlow -> apply(AnimationTypes.Fire);
      case Fire -> apply(AnimationTypes.Larson);
      case Larson -> apply(AnimationTypes.Rainbow);
      case Rainbow -> apply(AnimationTypes.RgbFade);
      case RgbFade -> apply(AnimationTypes.SingleFade);
      case SingleFade -> apply(AnimationTypes.Strobe);
      case Strobe -> apply(AnimationTypes.Twinkle);
      case Twinkle -> apply(AnimationTypes.TwinkleOff);
      case TwinkleOff -> apply(AnimationTypes.ColorFlow);
      case AutoAlign -> apply(AnimationTypes.Strobe);
      case SetAll -> apply(AnimationTypes.ColorFlow);
    }
  }

  public void decrementAnimation() {
    switch (current) {
      case ColorFlow -> apply(AnimationTypes.TwinkleOff);
      case Fire -> apply(AnimationTypes.ColorFlow);
      case Larson -> apply(AnimationTypes.Fire);
      case Rainbow -> apply(AnimationTypes.Larson);
      case RgbFade -> apply(AnimationTypes.Rainbow);
      case SingleFade -> apply(AnimationTypes.RgbFade);
      case Strobe -> apply(AnimationTypes.SingleFade);
      case Twinkle -> apply(AnimationTypes.Strobe);
      case TwinkleOff -> apply(AnimationTypes.Twinkle);
      case AutoAlign -> apply(AnimationTypes.Strobe);
      case SetAll -> apply(AnimationTypes.ColorFlow);
    }
  }

  @Override
  public void periodic() {}
}
