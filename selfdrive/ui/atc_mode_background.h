#pragma once

// The AUTO/NORM badge shares the existing TBT icon's actual blink phase.
// When the maneuver icon flashes, use yellow when the icon is visible and
// the mode's native background when it is hidden. Without icon blinking,
// preserve the old steady ATC-enabled color. No independent timer or
// separate turn/deceleration classification is used.
inline bool atc_mode_background_yellow(bool atc_enabled, bool icon_blink_active,
                                        bool icon_pulse_visible) {
  return icon_blink_active ? icon_pulse_visible : atc_enabled;
}
