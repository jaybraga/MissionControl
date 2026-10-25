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
#include "emulated_switch_controller.hpp"
#include "../bluetooth_mitm/bluetooth/bluetooth_hid.hpp"
#include "../mcmitm_config.hpp"

namespace ams::controller {

    EmulatedSwitchController::EmulatedSwitchController(bluetooth::Address address, HardwareID id) : SwitchController(address, id)
    , m_device_info(SwitchDevices::ProController)
    , m_joycon_emulation_type(JoyConEmulationType::ProController)
    , m_latency_timer(0)
    , m_input_report_mode(SwitchHidReportId::BasicInputReport)
    , m_vibration_processor(this)
    , m_hid_command_processor(this)
    , m_ext_grip_command_processor(this) {
        auto config = mitm::GetGlobalConfig();
        m_vibration_processor.SetEnabled(config->general.enable_rumble);
        m_enable_motion = config->general.enable_motion;
        m_trigger_threshold = config->misc.analog_trigger_activation_threshold / 100.0;

        // Start up already emulating the configured Joy-Con type, if any.
        switch (static_cast<JoyConEmulationType>(config->general.emulated_controller_type)) {
            case JoyConEmulationType::JoyConRight:
                m_joycon_emulation_type = JoyConEmulationType::JoyConRight;
                m_device_info = SwitchDevices::JoyConRight;
                break;
            case JoyConEmulationType::JoyConLeft:
                m_joycon_emulation_type = JoyConEmulationType::JoyConLeft;
                m_device_info = SwitchDevices::JoyConLeft;
                break;
            default:
                break;
        }
    };

    Result EmulatedSwitchController::Initialize() {
        R_TRY(SwitchController::Initialize());

        // Ensure config directory for this controller exists
        std::string controller_dir = GetControllerDirectory(m_address);
        R_TRY(fs::EnsureDirectory(controller_dir.c_str()));

        R_TRY(m_virtual_memory.Initialize((controller_dir + "/spi_flash.bin").c_str()));

        R_SUCCEED();
    }

    void EmulatedSwitchController::Reset() {
        m_buttons.Reset();
        m_buttons_prev.Reset();
        m_left_stick.Reset();
        m_right_stick.Reset();
        m_sixaxis_processor.Reset();
    }

    void EmulatedSwitchController::UpdateJoyConEmulationCombo() {
        if (!mitm::GetGlobalConfig()->general.enable_controller_type_combo)
            return;

        const SwitchButtons StickCombo = SwitchButton::StickL | SwitchButton::StickR;

        const bool combo_now  = m_buttons.All(StickCombo);
        const bool combo_prev = m_buttons_prev.All(StickCombo);
        m_buttons_prev = m_buttons;

        if (combo_now && !combo_prev) {
            switch (m_joycon_emulation_type) {
                case JoyConEmulationType::JoyConRight:
                    R_DISCARD(this->SetJoyConEmulationType(JoyConEmulationType::JoyConLeft));
                    break;
                case JoyConEmulationType::JoyConLeft:
                    R_DISCARD(this->SetJoyConEmulationType(JoyConEmulationType::ProController));
                    break;
                default:
                    R_DISCARD(this->SetJoyConEmulationType(JoyConEmulationType::JoyConRight));
                    break;
            }
        }
    }

    Result EmulatedSwitchController::SetJoyConEmulationType(JoyConEmulationType type) {
        if (type == m_joycon_emulation_type) {
            R_SUCCEED();
        }

        m_joycon_emulation_type = type;

        switch (type) {
            case JoyConEmulationType::JoyConRight:
                m_device_info = SwitchDevices::JoyConRight;
                break;
            case JoyConEmulationType::JoyConLeft:
                m_device_info = SwitchDevices::JoyConLeft;
                break;
            default:
                m_device_info = SwitchDevices::ProController;
                break;
        }

        // Make HOS drop and re-identify the controller so that it picks up the new
        // type from the GetDeviceInfo response, without actually losing the
        // underlying Bluetooth connection.
        R_RETURN(bluetooth::hid::VirtualReconnect(m_address));
    }

    void EmulatedSwitchController::ApplyJoyConEmulationButtons() {
        // A single Joy-Con is held horizontally, but its reports are always expressed
        // in the Joy-Con's own upright frame - HOS applies the horizontal rotation
        // itself. So the stick has to be rotated into that upright frame here, same
        // as the motion data in ApplyJoyConEmulationMotion.
        switch (m_joycon_emulation_type) {
            case JoyConEmulationType::JoyConRight:
                {
                    u16 x, y;
                    if (m_buttons.Any(SwitchButton::Up | SwitchButton::Down | SwitchButton::Left | SwitchButton::Right)) {
                        x = m_buttons.Test(SwitchButton::Down) ? SwitchAnalogStick::MaximumValue : (m_buttons.Test(SwitchButton::Up)   ? SwitchAnalogStick::MinimumValue : SwitchAnalogStick::CenterValue);
                        y = m_buttons.Test(SwitchButton::Right) ? SwitchAnalogStick::MaximumValue : (m_buttons.Test(SwitchButton::Left) ? SwitchAnalogStick::MinimumValue : SwitchAnalogStick::CenterValue);
                    } else {
                        x = SwitchAnalogStick::MaximumValue - m_left_stick.GetY();
                        y = m_left_stick.GetX();
                    }

                    // Write the rotated value into both stick fields so it is picked up
                    // regardless of which one HOS reads for this controller type.
                    m_left_stick.SetValues(x, y);
                    m_right_stick.SetValues(x, y);

                    SwitchButtons buttons = m_buttons;
                    buttons.Assign(SwitchButton::RightSL, m_buttons.Any(SwitchButton::L | SwitchButton::ZL));
                    buttons.Assign(SwitchButton::RightSR, m_buttons.Any(SwitchButton::R | SwitchButton::ZR));
                    buttons.Assign(SwitchButton::A, m_buttons.Test(SwitchButton::B));
                    buttons.Assign(SwitchButton::B, m_buttons.Test(SwitchButton::Y));
                    buttons.Assign(SwitchButton::X, m_buttons.Test(SwitchButton::A));
                    buttons.Assign(SwitchButton::Y, m_buttons.Test(SwitchButton::X));
                    m_buttons = buttons;
                }
                break;

            case JoyConEmulationType::JoyConLeft:
                {
                    u16 x, y;
                    if (m_buttons.Any(SwitchButton::Up | SwitchButton::Down | SwitchButton::Left | SwitchButton::Right)) {
                        x = m_buttons.Test(SwitchButton::Up)   ? SwitchAnalogStick::MaximumValue : (m_buttons.Test(SwitchButton::Down)  ? SwitchAnalogStick::MinimumValue : SwitchAnalogStick::CenterValue);
                        y = m_buttons.Test(SwitchButton::Left) ? SwitchAnalogStick::MaximumValue : (m_buttons.Test(SwitchButton::Right) ? SwitchAnalogStick::MinimumValue : SwitchAnalogStick::CenterValue);
                    } else {
                        x = m_left_stick.GetY();
                        y = SwitchAnalogStick::MaximumValue - m_left_stick.GetX();
                    }

                    m_left_stick.SetValues(x, y);
                    m_right_stick.SetValues(x, y);

                    SwitchButtons buttons = m_buttons;
                    buttons.Assign(SwitchButton::LeftSL, m_buttons.Any(SwitchButton::L | SwitchButton::ZL));
                    buttons.Assign(SwitchButton::LeftSR, m_buttons.Any(SwitchButton::R | SwitchButton::ZR));
                    buttons.Assign(SwitchButton::Down,  m_buttons.Test(SwitchButton::A));
                    buttons.Assign(SwitchButton::Left,  m_buttons.Test(SwitchButton::B));
                    buttons.Assign(SwitchButton::Up,    m_buttons.Test(SwitchButton::Y));
                    buttons.Assign(SwitchButton::Right, m_buttons.Test(SwitchButton::X));
                    m_buttons = buttons;
                }
                break;

            default:
                break;
        }
    }

    void EmulatedSwitchController::ApplyJoyConEmulationMotion(utils::Vec3d<float> *accel, utils::Vec3d<float> *gyro) {
        switch (m_joycon_emulation_type) {
            case JoyConEmulationType::JoyConRight:
                {
                    const float ax = accel->x;
                    accel->x = -accel->y;
                    accel->y = -ax;
                    accel->z = -accel->z;

                    const float gx = gyro->x;
                    gyro->x = -gyro->y;
                    gyro->y = -gx;
                    gyro->z = -gyro->z;
                }
                break;

            case JoyConEmulationType::JoyConLeft:
                {
                    const float ax = accel->x;
                    accel->x = accel->y;
                    accel->y = -ax;

                    const float gx = gyro->x;
                    gyro->x = gyro->y;
                    gyro->y = -gx;
                }
                break;

            default:
                break;
        }
    }

    Result EmulatedSwitchController::InjectInputReport(const u8 *report_buffer, size_t size) {
        bluetooth::hid::report::WriteHidDataReport(m_address, report_buffer, size);
        R_SUCCEED();
    }

    size_t EmulatedSwitchController::FillInputReport(u8 *report_buffer, size_t size) {
        switch (m_input_report_mode) {
            case SwitchHidReportId::BasicInputReport:
                return this->FillBasicInputReport(report_buffer, size);
            case SwitchHidReportId::McuInputReport:
                return this->FillMcuInputReport(report_buffer, size);
            case SwitchHidReportId::AttachmentInputReport:
                return this->FillAttachmentInputReport(report_buffer, size);
            case SwitchHidReportId::GenericInputReport:
                return this->FillGenericInputReport(report_buffer, size);
            default:
                return this->FillBasicInputReport(report_buffer, size);
        }
    }

    size_t EmulatedSwitchController::FillCommandInputReport(u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidCommandInputReport)); AMS_UNUSED(size);

        auto command_report = reinterpret_cast<SwitchHidCommandInputReport *>(report_buffer);
        command_report->report_id          = SwitchHidReportId::CommandInputReport;
        command_report->latency_timer      = m_latency_timer++;
        command_report->power_info         = m_power_info.GetState();
        command_report->buttons            = m_buttons.GetState();
        command_report->left_analog_stick  = m_left_stick.GetState();
        command_report->right_analog_stick = m_right_stick.GetState();
        command_report->motor_status       = m_motor_status.GetState();

        command_report->command_response = m_hid_command_processor.GetResponse();

        return sizeof(SwitchHidCommandInputReport);
    }

    size_t EmulatedSwitchController::FillMcuUpdateInputReport(u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchMcuUpdateInputReport)); AMS_UNUSED(size);

        auto mcu_update_report = reinterpret_cast<SwitchMcuUpdateInputReport *>(report_buffer);
        mcu_update_report->report_id          = SwitchHidReportId::McuUpdateInputReport;
        mcu_update_report->latency_timer      = m_latency_timer++;
        mcu_update_report->power_info         = m_power_info.GetState();
        mcu_update_report->buttons            = m_buttons.GetState();
        mcu_update_report->left_analog_stick  = m_left_stick.GetState();
        mcu_update_report->right_analog_stick = m_right_stick.GetState();
        mcu_update_report->motor_status       = m_motor_status.GetState();

        std::memset(&mcu_update_report->mcu_updata_data, 0, sizeof(mcu_update_report->mcu_updata_data));

        return sizeof(SwitchMcuUpdateInputReport);
    }

    size_t EmulatedSwitchController::FillBasicInputReport(u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidBasicInputReport)); AMS_UNUSED(size);

        auto basic_report = reinterpret_cast<SwitchHidBasicInputReport *>(report_buffer);
        basic_report->report_id          = SwitchHidReportId::BasicInputReport;
        basic_report->latency_timer      = m_latency_timer++;
        basic_report->power_info         = m_power_info.GetState();
        basic_report->buttons            = m_buttons.GetState();
        basic_report->left_analog_stick  = m_left_stick.GetState();
        basic_report->right_analog_stick = m_right_stick.GetState();
        basic_report->motor_status       = m_motor_status.GetState();

        basic_report->motion_data = m_sixaxis_processor.GetState();

        return sizeof(SwitchHidBasicInputReport);
    }

    size_t EmulatedSwitchController::FillMcuInputReport(u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidMcuInputReport)); AMS_UNUSED(size);

        auto mcu_report = reinterpret_cast<SwitchHidMcuInputReport *>(report_buffer);
        mcu_report->report_id          = SwitchHidReportId::McuInputReport;
        mcu_report->latency_timer      = m_latency_timer++;
        mcu_report->power_info         = m_power_info.GetState();
        mcu_report->buttons            = m_buttons.GetState();
        mcu_report->left_analog_stick  = m_left_stick.GetState();
        mcu_report->right_analog_stick = m_right_stick.GetState();
        mcu_report->motor_status       = m_motor_status.GetState();

        mcu_report->motion_data = m_sixaxis_processor.GetState();

        std::memset(&mcu_report->mcu_data, 0, sizeof(mcu_report->mcu_data));

        return sizeof(SwitchHidMcuInputReport);
    }

    size_t EmulatedSwitchController::FillAttachmentInputReport(u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidAttachmentInputReport)); AMS_UNUSED(size);

        auto attachment_report = reinterpret_cast<SwitchHidAttachmentInputReport *>(report_buffer);
        attachment_report->report_id          = SwitchHidReportId::AttachmentInputReport;
        attachment_report->latency_timer      = m_latency_timer++;
        attachment_report->power_info         = m_power_info.GetState();
        attachment_report->buttons            = m_buttons.GetState();
        attachment_report->left_analog_stick  = m_left_stick.GetState();
        attachment_report->right_analog_stick = m_right_stick.GetState();
        attachment_report->motor_status       = m_motor_status.GetState();

        attachment_report->motion_data = m_sixaxis_processor.GetState();

        std::memset(&attachment_report->attachment_data, 0, sizeof(attachment_report->attachment_data));

        return sizeof(SwitchHidAttachmentInputReport);
    }

    size_t EmulatedSwitchController::FillGenericInputReport(u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidGenericInputReport)); AMS_UNUSED(size);

        auto generic_report = reinterpret_cast<SwitchHidGenericInputReport *>(report_buffer);
        generic_report->report_id = SwitchHidReportId::GenericInputReport;

        // Todo: convert pad data to generic report format

        return sizeof(SwitchHidGenericInputReport);
    }

    size_t EmulatedSwitchController::FillExtGripInputReport(u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchExtGripInputReport)); AMS_UNUSED(size);

        auto ext_grip_report = reinterpret_cast<SwitchExtGripInputReport *>(report_buffer);
        ext_grip_report->report_id = SwitchHidReportId::ExtGripInputReport;

        ext_grip_report->command_response = m_ext_grip_command_processor.GetResponse();

        return sizeof(SwitchExtGripInputReport);
    }

    Result EmulatedSwitchController::HandleOutputDataReport(const u8 *report_buffer, size_t size) {
        auto report_id = static_cast<SwitchHidReportId>(report_buffer[0]);

        switch (report_id) {
            case SwitchHidReportId::CommandOutputReport:
                R_RETURN(this->HandleCommandOutputReport(report_buffer, size));

            case SwitchHidReportId::McuUpdateOutputReport:
                R_RETURN(this->HandleMcuUpdateOutputReport(report_buffer, size));

            case SwitchHidReportId::BasicOutputReport:
                R_RETURN(this->HandleBasicOutputReport(report_buffer, size));

            case SwitchHidReportId::McuOutputReport:
                R_RETURN(this->HandleMcuOutputReport(report_buffer, size));

            case SwitchHidReportId::AttachmentOutputReport:
                R_RETURN(this->HandleAttachmentOutputReport(report_buffer, size));

            case SwitchHidReportId::ExtGripOutputReport:
                R_RETURN(this->HandleExtGripOutputReport(report_buffer, size));

            AMS_UNREACHABLE_DEFAULT_CASE();
        }
    }

    Result EmulatedSwitchController::HandleCommandOutputReport(const u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidCommandOutputReport));

        auto command_report = reinterpret_cast<const SwitchHidCommandOutputReport *>(report_buffer);
        m_vibration_processor.ProcessMotorData(&command_report->motor_data);
        m_hid_command_processor.ProcessCommand(&command_report->command);

        u8 input_report[sizeof(SwitchHidCommandInputReport)];
        size = this->FillCommandInputReport(input_report, sizeof(input_report));
        R_TRY(this->InjectInputReport(input_report, size));

        R_SUCCEED();
    }

    Result EmulatedSwitchController::HandleMcuUpdateOutputReport(const u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidMcuUpdateOutputReport));

        auto mcu_update_report = reinterpret_cast<const SwitchHidMcuUpdateOutputReport *>(report_buffer);
        m_vibration_processor.ProcessMotorData(&mcu_update_report->motor_data);
        // Todo: process mcu update data

        u8 input_report[sizeof(SwitchMcuUpdateInputReport)];
        size = this->FillAttachmentInputReport(input_report, sizeof(input_report));
        R_TRY(this->InjectInputReport(input_report, size));

        R_SUCCEED();
    }

    Result EmulatedSwitchController::HandleBasicOutputReport(const u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidBasicOutputReport)); AMS_UNUSED(size);

        auto basic_report = reinterpret_cast<const SwitchHidBasicOutputReport *>(report_buffer);
        m_vibration_processor.ProcessMotorData(&basic_report->motor_data);

        R_SUCCEED();
    }

    Result EmulatedSwitchController::HandleMcuOutputReport(const u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidMcuOutputReport));

        auto mcu_report = reinterpret_cast<const SwitchHidMcuOutputReport *>(report_buffer);
        m_vibration_processor.ProcessMotorData(&mcu_report->motor_data);
        // m_mcu_command_processor.ProcessCommand(command);

        u8 input_report[sizeof(SwitchHidMcuInputReport)];
        size = this->FillMcuInputReport(input_report, sizeof(input_report));
        R_TRY(this->InjectInputReport(input_report, size));

        R_SUCCEED();
    }

    Result EmulatedSwitchController::HandleAttachmentOutputReport(const u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchHidAttachmentOutputReport));

        auto attachment_report = reinterpret_cast<const SwitchHidAttachmentOutputReport *>(report_buffer);
        m_vibration_processor.ProcessMotorData(&attachment_report->motor_data);
        // Todo: process attachment data

        u8 input_report[sizeof(SwitchHidAttachmentInputReport)];
        size = this->FillAttachmentInputReport(input_report, sizeof(input_report));
        R_TRY(this->InjectInputReport(input_report, size));

        R_SUCCEED();
    }

    Result EmulatedSwitchController::HandleExtGripOutputReport(const u8 *report_buffer, size_t size) {
        AMS_ASSERT(size >= sizeof(SwitchExtGripOutputReport));

        auto ext_grip_report = reinterpret_cast<const SwitchExtGripOutputReport *>(report_buffer);
        m_ext_grip_command_processor.ProcessCommand(&ext_grip_report->command);

        u8 input_report[sizeof(SwitchExtGripInputReport)];
        size = this->FillExtGripInputReport(input_report, sizeof(input_report));
        R_TRY(this->InjectInputReport(input_report, size));

        R_SUCCEED();
    }

}
