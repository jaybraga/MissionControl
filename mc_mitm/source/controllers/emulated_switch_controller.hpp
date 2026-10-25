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
#include "switch_controller.hpp"
#include "switch_device_info.hpp"
#include "switch_sixaxis_processor.hpp"
#include "switch_vibration_processor.hpp"
#include "switch_hid_command_processor.hpp"
#include "switch_mcu_command_processor.hpp"
#include "switch_ext_grip_command_processor.hpp"
#include "switch_virtual_controller_memory.hpp"

namespace ams::controller {

    class EmulatedSwitchController : public SwitchController {
        friend class SwitchVibrationProcessor;
        friend class SwitchHidCommandProcessor;
        friend class SwitchMcuCommandProcessor;
        friend class SwitchExtGripCommandProcessor;

        public:
            EmulatedSwitchController(bluetooth::Address address, HardwareID id);
            virtual ~EmulatedSwitchController() { };

        public:
            virtual Result Initialize() override;
            virtual constexpr bool IsOfficialController() const override { return false; }

            virtual JoyConEmulationType GetJoyConEmulationType() const override { return m_joycon_emulation_type; }

            virtual Result HandleOutputDataReport(const u8 *report_buffer, size_t size) override;
            virtual Result InjectInputReport(const u8 *report_buffer, size_t size);

        protected:
            void Reset();
            virtual Result SetVibration(const SwitchMotorData *motor_data) { AMS_UNUSED(motor_data); R_SUCCEED(); }
            virtual Result CancelVibration() { R_SUCCEED(); }
            virtual Result SetPlayerLed(SwitchPlayerNumber player_number) { AMS_UNUSED(player_number); R_SUCCEED(); }

            // Cycle ProController -> JoyConRight -> JoyConLeft -> ProController on a
            // rising edge of L3 + R3 (StickL + StickR), if enabled in config. Reads
            // and updates m_buttons/m_buttons_prev; call after m_buttons has been
            // assigned for the current report.
            void UpdateJoyConEmulationCombo();

            // Change the controller identity (device info) reported to HOS and force
            // it to re-identify the controller without dropping the Bluetooth link.
            Result SetJoyConEmulationType(JoyConEmulationType type);

            // Rewrite m_buttons/m_left_stick/m_right_stick (already populated for the
            // current report) into the single-Joy-Con layout, if m_joycon_emulation_type
            // is not ProController. No-op otherwise.
            void ApplyJoyConEmulationButtons();

            // Rotate an accelerometer/gyroscope sample pair from the Pro Controller
            // frame into the single-Joy-Con's own upright frame. Call before handing
            // accel/gyro to the sixaxis processor. No-op if not emulating a Joy-Con.
            void ApplyJoyConEmulationMotion(utils::Vec3d<float> *accel, utils::Vec3d<float> *gyro);

            virtual size_t FillInputReport(u8 *report_buffer, size_t size) override;

            size_t FillCommandInputReport(u8 *report_buffer, size_t size);
            size_t FillMcuUpdateInputReport(u8 *report_buffer, size_t size);
            size_t FillBasicInputReport(u8 *report_buffer, size_t size);
            size_t FillMcuInputReport(u8 *report_buffer, size_t size);
            size_t FillAttachmentInputReport(u8 *report_buffer, size_t size);
            size_t FillGenericInputReport(u8 *report_buffer, size_t size);
            size_t FillExtGripInputReport(u8 *report_buffer, size_t size);

            Result HandleCommandOutputReport(const u8 *report_buffer, size_t size);
            Result HandleMcuUpdateOutputReport(const u8 *report_buffer, size_t size);
            Result HandleBasicOutputReport(const u8 *report_buffer, size_t size);
            Result HandleMcuOutputReport(const u8 *report_buffer, size_t size);
            Result HandleAttachmentOutputReport(const u8 *report_buffer, size_t size);
            Result HandleExtGripOutputReport(const u8 *report_buffer, size_t size);

        protected:
            SwitchDeviceInfo m_device_info;

            JoyConEmulationType m_joycon_emulation_type;
            SwitchButtons m_buttons_prev;

            u8 m_latency_timer;
            SwitchPowerInfo m_power_info;
            SwitchPlayerIndicator m_player_indicator;
            SwitchMotorStatus m_motor_status;
            SwitchHidReportId m_input_report_mode;

            float m_trigger_threshold;

            bool m_enable_motion;
            SwitchSixAxisProcessor m_sixaxis_processor;

            SwitchVibrationProcessor m_vibration_processor;
            SwitchHidCommandProcessor m_hid_command_processor;
            SwitchMcuCommandProcessor m_mcu_command_processor;
            SwitchExtGripCommandProcessor m_ext_grip_command_processor;

            SwitchVirtualControllerMemory m_virtual_memory;
    };

}
