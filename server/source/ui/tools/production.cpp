/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "ui/ui.hpp"

#include "device/tracking_camera.hpp"
#include "device/tracking_controller.hpp"

#include "comm/serial.hpp"
#include "comm/commands.h"

#include <random>

const uint8_t OTP_Version = 1; // Sync with OTP_Version_Latest, as well as below properties
const uint8_t OTP_MaxSubpartBlocks = 8;
const auto OTP_MaxHwDescBlocks = [](int OTP_Version){ return OTP_Version == 1? 100 : 0; };

static void ShowHardwareSerial(HardwareSerial hwSerial);
static void EditHardwareSerial(HardwareSerial &hwSerial);
static int EditHardwareDescriptor(std::string &hwDesc);
static int EditHardwareSubparts(std::vector<uint64_t> &hwSubparts, const std::vector<uint64_t> *exSubparts = nullptr);

static bool ProgramInitialMainOTP(TrackingCameraState &camera, HardwareSerial hwSerial)
{
	static std::mt19937 gen = std::mt19937(std::random_device{}());
	hwSerial.ID = gen();

	std::vector<uint8_t> request(4 + 3*8);
	request[0] = 1; // Program main OTP
	uint32_t *otpPtr = (uint32_t*)&request[4];
	uint64_t OTP_HEADER = (uint64_t)OTP_Version << 56;
	otpPtr[0] = OTP_HEADER >> 32;
	otpPtr[1] = OTP_HEADER & 0xFFFFFFFF;
	otpPtr[2] = hwSerial.serial32[0];
	otpPtr[3] = hwSerial.serial32[1];
	otpPtr[4] = hwSerial.serial32[2];
	otpPtr[5] = 0;
	return camera.sendPacket(PACKET_WRITE_MCU_INFO, request.data(), request.size());
}

static bool ProgramSetHWSubparts(TrackingCameraState &camera, const std::vector<uint64_t> &hwSubparts)
{
	std::vector<uint8_t> request(4 + hwSubparts.size()*sizeof(uint32_t));
	request[0] = 2; // Program subparts into OTP
	uint32_t *otpPtr = (uint32_t*)&request[4];
	memcpy(otpPtr, hwSubparts.data(), hwSubparts.size()*sizeof(uint64_t));
	return camera.sendPacket(PACKET_WRITE_MCU_INFO, request.data(), request.size());
}

static bool ProgramAppendHWString(TrackingCameraState &camera, const std::string &hwDescriptor)
{
	std::vector<uint8_t> request(4 + hwDescriptor.size());
	request[0] = 3; // Program hw string into OTP
	memcpy(request.data()+4, hwDescriptor.data(), hwDescriptor.size());
	return camera.sendPacket(PACKET_WRITE_MCU_INFO, request.data(), request.size());
}

void InterfaceState::UpdateProductionTool(InterfaceWindow &window)
{
	if (!window.open)
		return;
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}

	ServerState &state = GetState();

	// TODO: Support cameras that were not flashed, requires controller cooperation (probing for bootloader)
	// TODO: Support cameras with no ID in config (never connected to SBC, appears as newCamerasConnecting)

	ImGui::SeparatorText("Connected Cameras");
	static CameraID selectedCamera = 0;
	std::shared_ptr<TrackingCameraState> selCamera = nullptr;
	for (auto &camera : state.cameras)
	{
		if (!camera->controller) continue;
		std::string label = asprintf_s("Controller %d, Port %d: Camera #%u", camera->controller->id, camera->port, camera->id);
		if (ImGui::Selectable(label.c_str(), camera->id == selectedCamera, ImGuiSelectableFlags_SpanAvailWidth))
		{
			if (camera->id == selectedCamera) selectedCamera = 0;
			else selectedCamera = camera->id;
			if (!camera->storage.receivedInfo && !camera->storage.receivedMCUInfo)
			{ // Request info from MCU
				CameraRequestMCUInfo(*camera);
			}
		}
		if (camera->id == selectedCamera)
		{
			selCamera = camera;
			if (!camera->storage.receivedInfo && !camera->storage.receivedMCUInfo)
			{
				SameLineTrailing(ImGui::GetFrameHeight());
				ImGui::Text("...");
			}
		}
	}
	if (selectedCamera != 0 && !selCamera)
		selectedCamera = 0;
	ImGui::Spacing();

	// Currently editing initial OTP config
	static HardwareSerial hwSerial = {
		.header = HardwareSerial::Class::Camera,
		.type = HardwareSerial::Type::Official,
		.manufacturer = 0xF8,
		.camera = {
			.interface = HardwareSerial::Camera::Interface::RS422_WIFI_NRF24,
			.sensor = HardwareSerial::Camera::Sensor::OV9281,
			.lens = HardwareSerial::Camera::Lens::HSW_9D4024,
			.filter = HardwareSerial::Camera::Filter::Switched_850nm_IRCut,
		},
	};
	static std::vector<uint64_t> hwSubparts;
	static std::string hwDescriptor = "AsterTrack Camera V1.1";

	if (ImGui::CollapsingHeader("Initial OTP Config"))
	{
		ImGui::BeginDisabled(true);
		int v = OTP_Version;
		ScalarProperty<int>("OTP Version", "", &v, nullptr, 1, v);
		ImGui::EndDisabled();

		ImGui::Text("Serial: ");
		ImGui::SameLine();
		ShowHardwareSerial(hwSerial);
		//ImGui::Text("%s", PrintHardwareSerial(hwSerial).c_str());

		ImGui::AlignTextToFramePadding();
		if (ImGui::TreeNode("Hardware Serial"))
		{
			EditHardwareSerial(hwSerial);
			ImGui::TreePop();
		}

		ImGui::AlignTextToFramePadding();
		if (ImGui::TreeNode("Hardware Subparts"))
		{
			EditHardwareSubparts(hwSubparts);
			ImGui::TreePop();
		}

		ImGui::AlignTextToFramePadding();
		if (ImGui::TreeNode("Hardware Descriptor"))
		{
			int blocks = EditHardwareDescriptor(hwDescriptor);
			ImGui::Text("HW Descriptor will occupy %d / %d blocks.", blocks, OTP_MaxHwDescBlocks(OTP_Version));
			ImGui::TreePop();
		}

		bool canModCamera = selCamera && (selCamera->storage.receivedInfo || selCamera->storage.receivedMCUInfo);
		bool otpInitialised = selCamera && selCamera->storage.info.mcuOTPVersion != 0xFF; // OTP_Version_NotWritten

		if (!canModCamera)
			ImGui::Text("No camera selected.");
		else if (otpInitialised)
			ImGui::Text("OTP of selected camera is already initialised (version %d)!", selCamera->storage.info.mcuOTPVersion);
		else
			ImGui::Text("OTP of selected camera is not yet initialised!");

		ImGui::BeginDisabled(!canModCamera);
		// Allow writing even if already initialised for MCU testing FW using Flash instead of OTP
		if (ImGui::Button("Permanently Initialise OTP", SizeWidthFull()))
		{
			if (!ProgramInitialMainOTP(*selCamera, hwSerial))
				SignalErrorToUser("Failed to send message to program Main OTP!");
			else if (!ProgramSetHWSubparts(*selCamera, hwSubparts))
				SignalErrorToUser("Failed to send message to program subparts to OTP!");
			else if (!ProgramAppendHWString(*selCamera, hwDescriptor))
				SignalErrorToUser("Failed to send message to program hardware descriptor!");
			selCamera->storage.receivedInfo = selCamera->storage.receivedMCUInfo = false;
			CameraRequestMCUInfo(*selCamera);
		}
		ImGui::EndDisabled();
	}

	if (ImGui::CollapsingHeader("Set Additional Subparts"))
	{
		ImGui::TextWrapped("Set subpart entries not already written in OTP - this cannot be undone!");

		static std::vector<uint64_t> appendHWSubparts;
		int writing = EditHardwareSubparts(appendHWSubparts);

		bool canModCamera = selCamera && (selCamera->storage.receivedInfo || selCamera->storage.receivedMCUInfo);

		ImGui::BeginDisabled(!canModCamera || writing == 0);
		if (ImGui::Button("Permanently Set HW Subparts", SizeWidthFull()))
		{
			if (!ProgramSetHWSubparts(*selCamera, appendHWSubparts))
				SignalErrorToUser("Failed to send message to set additional hardware subparts!");
			selCamera->storage.receivedInfo = selCamera->storage.receivedMCUInfo = false;
			CameraRequestMCUInfo(*selCamera);
		}
		ImGui::EndDisabled();
	}

	if (ImGui::CollapsingHeader("Append Hardware Descriptor"))
	{
		ImGui::TextWrapped("Append to the string descriptor stored in OTP - this cannot be undone! "
			"Use for documenting repairs or changes done to hardware.");

		static std::string appendHWDescriptor = "";
		int blocksAppend = EditHardwareDescriptor(appendHWDescriptor);
		ImGui::Text("HW Descriptor will be expanded by %d blocks.", blocksAppend);

		bool canModCamera = selCamera && (selCamera->storage.receivedInfo || selCamera->storage.receivedMCUInfo);
		int blocksUsed = 0, blocksMax = 0;
		if (canModCamera)
		{
			blocksMax = OTP_MaxHwDescBlocks(selCamera->storage.info.mcuOTPVersion);
			for (auto &str : selCamera->storage.info.mcuHWDescriptorParts)
				blocksUsed += ((str.size()+1) + 7) / 8; // Add space for trailing \0, then round up
		}

		if (!canModCamera)
			ImGui::Text("No camera selected.");
		else if (blocksUsed + blocksAppend < blocksMax)
			ImGui::Text("Existing HW descriptor uses %d / %d blocks.", blocksUsed, blocksMax);
		else
			ImGui::Text("Existing HW descriptor already uses %d / %d blocks!", blocksUsed, blocksMax);

		ImGui::BeginDisabled(!canModCamera || blocksUsed + blocksAppend >= blocksMax);
		if (ImGui::Button("Permanently Append to HW Descriptor", SizeWidthFull()))
		{
			if (!ProgramAppendHWString(*selCamera, appendHWDescriptor))
				SignalErrorToUser("Failed to send message to append to hardware descriptor!");
			selCamera->storage.receivedInfo = selCamera->storage.receivedMCUInfo = false;
			CameraRequestMCUInfo(*selCamera);
		}
		ImGui::EndDisabled();
	}

	ImGui::End();
}

static void ShowHardwareSerial(HardwareSerial hwSerial)
{
	ImGui::Text("%.1X", (uint8_t)hwSerial.header);
	ImGui::SetItemTooltip("Class of Product : 4");
	ImGui::SameLine(0, 2.0f);
	ImGui::Text("%.1X", (uint8_t)hwSerial.type);
	ImGui::SetItemTooltip("Type of Producer : 4");
	ImGui::SameLine(0, 2.0f);

	ImGui::Text("%.2X", (uint8_t)hwSerial.manufacturer);
	ImGui::SetItemTooltip("Manufacturer / Facility : 8");
	ImGui::SameLine(0, 2.0f);

	ImGui::Text("%.2X", (uint8_t)hwSerial.product);
	ImGui::SetItemTooltip("Product Number : 8");
	ImGui::SameLine(0, 2.0f);
	ImGui::Text("%.2X", (uint8_t)hwSerial.revision);
	ImGui::SetItemTooltip("Revision Number : 8");
	ImGui::SameLine(0, 3.0f);

	ImGui::Text("%.4x", (uint16_t)hwSerial.config);
	ImGui::SetItemTooltip("Configuration Field : 16");
	ImGui::SameLine(0, 3.0f);

	ImGui::Text("%.2X", (uint8_t)hwSerial.batch);
	ImGui::SetItemTooltip("Production Batch : 8");
	ImGui::SameLine(0, 2.0f);
	ImGui::Text("%.2X", (uint8_t)hwSerial.run);
	ImGui::SetItemTooltip("Run within Batch : 8");
	ImGui::SameLine(0, 3.0f);

	ImGui::Text("%.8X", hwSerial.ID);
	ImGui::SetItemTooltip("Random ID : 32");
}

static void EditHardwareSerial(HardwareSerial &hwSerial)
{
	int header = (int)hwSerial.header;
	BeginLabelledGroup("Class");
	if (ImGui::Combo("##Class", &header, "Controller\0Sync Beacon\0Camera\0Tracker\0\0"))
		hwSerial.header = (HardwareSerial::Class)header;
	ImGui::EndGroup();

	int type = (int)hwSerial.type;
	BeginLabelledGroup("Type");
	if (ImGui::Combo("##Type", &type, "Official\0Third-Party\0DIY (Registered 1)\0DIY (Open 1)\0\0"))
		hwSerial.type = (HardwareSerial::Type)type;
	ImGui::EndGroup();

	int manufacturer = (int)hwSerial.manufacturer;
	if (ScalarProperty<int>("Manufacturer", "", &manufacturer, nullptr, 0x00, 0xFF))
		hwSerial.manufacturer = manufacturer;

	int product = (int)hwSerial.product;
	if (ScalarProperty<int>("Product", "", &product, nullptr, 0x00, 0xFF))
		hwSerial.product = product;

	int revision = (int)hwSerial.revision;
	if (ScalarProperty<int>("Revision", "", &revision, nullptr, 0x00, 0xFF))
		hwSerial.revision = revision;

	if (hwSerial.header == HardwareSerial::Class::Camera)
	{
		int interface = (int)hwSerial.camera.interface;
		BeginLabelledGroup("Interface");
		if (ImGui::Combo("##Interface", &interface, "Unknown\0RS422/Wifi/nRF24\0\0"))
			hwSerial.camera.interface = (HardwareSerial::Camera::Interface)interface;
		ImGui::EndGroup();

		int sensor = (int)hwSerial.camera.sensor;
		BeginLabelledGroup("Sensor");
		if (ImGui::Combo("##Sensor", &sensor, "Unknown\0OV9281\0\0"))
			hwSerial.camera.sensor = (HardwareSerial::Camera::Sensor)sensor;
		ImGui::EndGroup();

		int lens = (int)hwSerial.camera.lens;
		BeginLabelledGroup("Default Lens");
		if (ImGui::Combo("##Default Lens", &lens, "Unknown\0HSW 9D4024\0\0"))
			hwSerial.camera.lens = (HardwareSerial::Camera::Lens)lens;
		ImGui::EndGroup();

		int filter = (int)hwSerial.camera.filter;
		BeginLabelledGroup("Filter");
		if (ImGui::Combo("##Filter", &filter, "Unknown\0None\0IR Cut\0850nm\0Switched (850nm, IR Cut)\0\0"))
			hwSerial.camera.filter = (HardwareSerial::Camera::Filter)filter;
		ImGui::EndGroup();
	}
	else if (hwSerial.header == HardwareSerial::Class::Controller)
	{
		int interface = (int)hwSerial.controller.interface;
		BeginLabelledGroup("Interface");
		if (ImGui::Combo("##Interface", &interface, "Unknown\0USB 2.0 HS/RS422/nRF24\0\0"))
			hwSerial.camera.interface = (HardwareSerial::Camera::Interface)interface;
		ImGui::EndGroup();

		int ports = (int)hwSerial.controller.ports + 1;
		if (ScalarProperty<int>("Ports", "", &ports, nullptr, 1, 16))
			hwSerial.controller.ports = ports-1;
	}

	int batch = (int)hwSerial.batch;
	if (ScalarProperty<int>("Batch", "", &batch, nullptr, 0x00, 0xFF))
		hwSerial.batch = batch;

	int run = (int)hwSerial.run;
	if (ScalarProperty<int>("Run", "", &run, nullptr, 0x00, 0xFF))
		hwSerial.run = run;
}

static int EditHardwareDescriptor(std::string &hwDesc)
{
	ImGui::SetNextItemWidth(SizeWidthFull().x);
	ImGui::InputText("##hwDesc", &hwDesc);
	if (hwDesc.contains(MCU_MULTI_TEXT_SEP)) // Disallow
		hwDesc.resize(hwDesc.find_first_of(MCU_MULTI_TEXT_SEP));
	if (hwDesc.contains('\0')) // Disallow
		hwDesc.resize(hwDesc.find_first_of('\0'));
	return ((hwDesc.size()+1) + 7) / 8; // Add space for trailing \0, then round up
}

static int EditHardwareSubparts(std::vector<uint64_t> &hwSubparts, const std::vector<uint64_t> *exSubparts)
{
	int count = 0;
	for (int i = 0; i < hwSubparts.size(); i++)
	{
		bool existing = exSubparts && exSubparts->size() > i && exSubparts->at(i) != (uint64_t)-1;
		ImGui::PushID(i);
		ImGui::BeginDisabled(existing);
		ImGui::SetNextItemWidth(SizeWidthFull().x);
		ImGui::InputScalarN("##subpart", ImGuiDataType_U32, (uint32_t*)&hwSubparts[i], 2, NULL, NULL, "%.8X");
		ImGui::EndDisabled();
		if (existing)
			ImGui::SetItemTooltip("This Subpart Entry has already been programmed!");
		else if (hwSubparts[i] != (uint64_t)-1)
			count++;
		ImGui::PopID();
	}
	ImGui::BeginGroup();
	ImGui::AlignTextToFramePadding();
	ImGui::Text("Writing %d / %d", (int)count, OTP_MaxSubpartBlocks);
	SameLineTrailing(GetBarWidth(ImGui::GetFrameHeight(), 2));

	ImGui::BeginDisabled(hwSubparts.empty());
	if (ImGui::Button("-", ImVec2(ImGui::GetFrameHeight(), 0)))
		hwSubparts.resize(hwSubparts.size()-1);
	ImGui::EndDisabled();
	ImGui::SameLine();

	ImGui::BeginDisabled(hwSubparts.size() >= OTP_MaxSubpartBlocks);
	if (ImGui::Button("+", ImVec2(ImGui::GetFrameHeight(), 0)))
		hwSubparts.resize(hwSubparts.size()+1, (uint64_t)-1);
	ImGui::EndDisabled();

	ImGui::EndGroup();
	ImGui::SetItemTooltip("Subparts are optional slots for hardware components, stored in 2 32-Bit pairs.\n"
		"Position of subpart determines what it describes, so use new slots carefully!\n"
		"All bits set prevents writing, leaving this slot free to be written later.\n"
		"All bits 0 explicitly disables slot permanently.");
	return count;
}
