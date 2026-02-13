// Team: FRC 1721 - Concord Robotics (Tidal Force)
// Year: 2025-2026
// Code: Public codebase for our REBUILT frc robot
// License: MIT License (See LICENSE file for full text)
//
// Copyright (c) 2025-2026 Concord Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN an ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package org.tidalforce.frc2026.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj.util.Color;
import java.util.List;
import org.tidalforce.frc2026.Constants;
import org.tidalforce.frc2026.Ports;
import org.tidalforce.frc2026.util.Device;

public class LEDsConstants {
  public static final String NAME = "MainLEDs";
  public static final Device.CAN CANDLE_ID = new Device.CAN(30, "rio");

  public static final LEDSegment CANDLE_LEDS = new LEDSegment(0, 7, 0);
  public static final LEDSegment FRONT_STRIP = new LEDSegment(8, 10, 1);

  public static final CANdleConfiguration CANDLE_CONFIG =
      new CANdleConfiguration()
          .withCANdleFeatures(
              new CANdleFeaturesConfigs()
                  .withEnable5VRail(Enable5VRailValue.Enabled)
                  .withVBatOutputMode(VBatOutputModeValue.On)
                  .withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled))
          .withLED(
              new LEDConfigs()
                  .withBrightnessScalar(1.0)
                  .withStripType(StripTypeValue.RGB)
                  .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs));

  public static final LightsIOCandle getLightsIOReal() {
    return new LightsIOCandle(NAME, Ports.lights, CANDLE_CONFIG);
  }

  public static final LightsIOSim getLightsIOSim() {
    return new LightsIOSim(NAME);
  }

  public static final LightsIO getLightsIOReplay() {
    return new LightsIO() {};
  }

  public static LEDs get() {
    switch (Constants.robot) {
      case DEV:
        return new LEDs(new LightsIOCandle(NAME, Ports.lights, CANDLE_CONFIG));
      case SIM:
        return new LEDs(new LightsIOSim(NAME));
      case COMP:
        return new LEDs(new LightsIO() {});
      default:
        throw new IllegalStateException("Unrecognized Robot Mode");
    }
  }

  public record LEDSegment(int startIndex, int endIndex, int animationSlot) {}
  ;

  // Animations

  // Off
  public static final ControlRequest candleOff = new EmptyAnimation(CANDLE_LEDS.animationSlot);

  public static final ControlRequest frontOff = new EmptyAnimation(FRONT_STRIP.animationSlot);
  ;

  public static final List<ControlRequest> offAnimation = List.of(candleOff, frontOff);

  // Disabled
  public static final ControlRequest candleDisabled =
      new RainbowAnimation(CANDLE_LEDS.startIndex, CANDLE_LEDS.endIndex)
          .withSlot(CANDLE_LEDS.animationSlot)
          .withFrameRate(10)
          .withBrightness(0.7)
          .withDirection(AnimationDirectionValue.Forward);

  public static final ControlRequest frontDisabled =
      new RainbowAnimation(FRONT_STRIP.startIndex, FRONT_STRIP.endIndex)
          .withSlot(FRONT_STRIP.animationSlot)
          .withFrameRate(10)
          .withBrightness(0.7)
          .withDirection(AnimationDirectionValue.Forward);

  public static final List<ControlRequest> disabledAnimation =
      List.of(candleDisabled, frontDisabled);

  // Auto
  public static final ControlRequest candleAuto =
      new FireAnimation(CANDLE_LEDS.startIndex, CANDLE_LEDS.endIndex)
          .withSlot(CANDLE_LEDS.animationSlot)
          .withFrameRate(10)
          .withBrightness(0.7)
          .withDirection(AnimationDirectionValue.Forward)
          .withCooling(0.1)
          .withSparking(1.0);

  public static final ControlRequest frontAuto =
      new FireAnimation(FRONT_STRIP.startIndex, FRONT_STRIP.endIndex)
          .withSlot(FRONT_STRIP.animationSlot)
          .withFrameRate(10)
          .withBrightness(0.7)
          .withDirection(AnimationDirectionValue.Forward)
          .withCooling(0.1)
          .withSparking(1.0);

  public static final List<ControlRequest> autoAnimation = List.of(candleAuto, frontAuto);

  // Flashing
  public static final ControlRequest candleFlash =
      new StrobeAnimation(CANDLE_LEDS.startIndex, CANDLE_LEDS.endIndex)
          .withSlot(CANDLE_LEDS.animationSlot)
          .withFrameRate(10)
          .withColor(new RGBWColor(Color.kRed));
}
