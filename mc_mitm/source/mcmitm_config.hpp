/*
 * Copyright (c) 2020-2026 ndeadly
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
#include "bluetooth_mitm/bluetooth/bluetooth_types.hpp"

namespace ams::mitm {

    struct MissionControlConfig {
        struct {
            bool enable_rumble;
            bool enable_motion;
            // Joy-Con emulation type to report for unofficial controllers at startup.
            // See controller::JoyConEmulationType in controllers/switch_controller.hpp.
            int emulated_controller_type;
            // Allow cycling the emulated controller type at runtime with L3 + R3.
            bool enable_controller_type_combo;
        } general;

        struct {
            char host_name[0x20];
            bluetooth::Address host_address;
        } bluetooth;

        struct {
            int analog_trigger_activation_threshold;
            bool dualshock3_enable_usb_pairing;
            int dualshock3_led_mode;
            int dualshock4_polling_rate;
            int dualshock4_lightbar_brightness;
            int dualsense_lightbar_brightness;
            bool dualsense_enable_player_leds;
            int dualsense_vibration_intensity;
            int dualsense_adaptive_trigger_travel;
        } misc;
    };

    void LoadConfiguration();
    MissionControlConfig *GetGlobalConfig();
    SetLanguage GetSystemLanguage();

}
