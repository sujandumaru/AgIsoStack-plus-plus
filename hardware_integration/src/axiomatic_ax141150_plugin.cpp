//================================================================================================
/// @file axiomatic_ax141150_plugin.cpp
///
/// @brief A CAN driver for the Axiomatic AX141150 CAN-Bluetooth converter (CAN2BT).
/// @author Open-Agriculture contributors
///
/// @copyright 2026 The Open-Agriculture Developers
//================================================================================================
#include "isobus/hardware_integration/axiomatic_ax141150_plugin.hpp"
#include "isobus/isobus/can_constants.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/utility/system_timing.hpp"
#include "isobus/utility/to_string.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <limits>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

#ifdef __linux__
#include <sys/socket.h>
#include <cstddef>
#endif

namespace isobus
{
	constexpr std::size_t CRC_LENGTH = 4; ///< Bytes of CRC32 at the end of each SPP message
	constexpr std::size_t HEADER_LENGTH = 2; ///< Type byte and length byte
	constexpr std::size_t MAX_MESSAGE_LENGTH = std::numeric_limits<std::uint8_t>::max(); ///< The length field is one byte
	constexpr std::size_t READ_CHUNK_SIZE = 512; ///< Bytes read from the converter per call
	constexpr std::size_t MAX_RECEIVE_BUFFER_SIZE = 4096; ///< Guard against unbounded growth of the receive buffer
	constexpr std::size_t STANDARD_IDENTIFIER_LENGTH = 2; ///< Bytes of identifier in a standard frame record
	constexpr std::size_t EXTENDED_IDENTIFIER_LENGTH = 4; ///< Bytes of identifier in an extended frame record
	constexpr std::size_t MAX_FRAME_RECORD_LENGTH = 1 + EXTENDED_IDENTIFIER_LENGTH + 1 + CAN_DATA_LENGTH; ///< Type, identifier, data length and data of one record
	constexpr std::size_t MAX_FRAMES_PER_MULTIPLE_FRAMES_MESSAGE = 6; ///< Frame records in one MultipleCANFramesFromDevice message
	constexpr std::uint32_t STANDARD_IDENTIFIER_MASK = 0x7FF; ///< The 11 identifier bits of a standard frame
	constexpr std::uint32_t EXTENDED_IDENTIFIER_MASK = 0x1FFFFFFF; ///< The 29 identifier bits of an extended frame
	constexpr std::uint32_t STANDARD_RTR_FLAG = 0x0800; ///< Bit 11 of a standard identifier field
	constexpr std::uint32_t EXTENDED_RTR_FLAG = 0x80000000; ///< Bit 31 of an extended identifier field
	constexpr std::uint32_t BITS_PER_BYTE = std::numeric_limits<std::uint8_t>::digits; ///< Shift per byte of a little endian field
	constexpr std::uint32_t CRC32_INITIAL_VALUE = 0xFFFFFFFF; ///< The zlib CRC32 initial value
	constexpr std::uint32_t CRC32_POLYNOMIAL = 0xEDB88320; ///< The reflected zlib CRC32 polynomial
	constexpr std::uint32_t RECEIVE_TIMEOUT_MS = 100; ///< How long one read waits for data
	constexpr std::uint32_t WRITE_TIMEOUT_MS = 1000; ///< How long one write may block on Windows
	constexpr std::uint32_t ACKNOWLEDGEMENT_POLL_INTERVAL_MS = 50; ///< How long one read waits while waiting for a configuration acknowledgement
	constexpr std::size_t BLUETOOTH_ADDRESS_LENGTH = 6; ///< Bytes in a Bluetooth address
	constexpr std::size_t BLUETOOTH_ADDRESS_TEXT_STRIDE = 3; ///< Two hex digits and a colon per byte in the text form
	constexpr std::size_t BLUETOOTH_ADDRESS_TEXT_LENGTH = (BLUETOOTH_ADDRESS_LENGTH * BLUETOOTH_ADDRESS_TEXT_STRIDE) - 1; ///< "XX:XX:XX:XX:XX:XX"
	constexpr std::uint8_t MIN_RFCOMM_CHANNEL = 1; ///< The lowest valid RFCOMM channel
	constexpr std::uint8_t MAX_RFCOMM_CHANNEL = 30; ///< The highest valid RFCOMM channel
	constexpr std::uint8_t BITS_PER_NIBBLE = 4; ///< Bits per hex digit
	constexpr std::uint8_t NIBBLE_MASK = 0x0F; ///< The low hex digit of a byte

	/// @brief Returns true for message types the converter sends to the host, so the decoder can resynchronise quickly
	/// @param[in] type The message type byte
	/// @returns `true` if the converter sends this message type
	static bool is_known_device_message_type(std::uint8_t type)
	{
		bool known = false;
		switch (static_cast<AX141150Plugin::MessageType>(type))
		{
			case AX141150Plugin::MessageType::CANDataStandardFromDevice:
			case AX141150Plugin::MessageType::CANDataExtendedFromDevice:
			case AX141150Plugin::MessageType::MapWriteResponse:
			case AX141150Plugin::MessageType::MapReadResponse:
			case AX141150Plugin::MessageType::DownloadFlashResponse:
			case AX141150Plugin::MessageType::DownloadFlashAcknowledgeResponse:
			case AX141150Plugin::MessageType::ChangeConfigurationResponse:
			case AX141150Plugin::MessageType::LEDStateResponse:
			case AX141150Plugin::MessageType::RequestPGNResponse:
			case AX141150Plugin::MessageType::MultipleCANFramesFromDevice:
			{
				known = true;
			}
			break;

			default:
				break;
		}
		return known;
	}

	/// @brief Returns the identifier length of a CAN frame record type
	/// @param[in] recordType The message or record type byte
	/// @returns The identifier length in bytes, or 0 if the type is not a CAN frame
	static std::size_t get_identifier_length(std::uint8_t recordType)
	{
		std::size_t identifierLength = 0;
		if (static_cast<std::uint8_t>(AX141150Plugin::MessageType::CANDataStandardFromDevice) == recordType)
		{
			identifierLength = STANDARD_IDENTIFIER_LENGTH;
		}
		else if (static_cast<std::uint8_t>(AX141150Plugin::MessageType::CANDataExtendedFromDevice) == recordType)
		{
			identifierLength = EXTENDED_IDENTIFIER_LENGTH;
		}
		return identifierLength;
	}

	/// @brief Checks if a length byte is possible for a message type, so a false start doesn't stall the decoder
	/// waiting for bytes that will never arrive
	/// @param[in] message The buffer at the candidate start, at least HEADER_LENGTH bytes
	/// @param[in] available Number of bytes available at `message`
	/// @returns `true` if the length is possible
	static bool is_plausible_length(const std::uint8_t *message, std::size_t available)
	{
		const std::uint8_t type = message[0];
		const std::size_t length = message[1];
		const std::size_t identifierLength = get_identifier_length(type);
		bool plausible = (length >= HEADER_LENGTH);

		if (plausible && (static_cast<std::uint8_t>(AX141150Plugin::MessageType::MultipleCANFramesFromDevice) == type))
		{
			plausible = (length <= (HEADER_LENGTH + (MAX_FRAMES_PER_MULTIPLE_FRAMES_MESSAGE * MAX_FRAME_RECORD_LENGTH)));
		}
		else if (plausible && (0 != identifierLength))
		{
			// A single frame's length is fully determined by its data length byte
			const std::size_t dataLengthIndex = HEADER_LENGTH + identifierLength;
			plausible = (length > dataLengthIndex) &&
			  (length <= (dataLengthIndex + 1 + CAN_DATA_LENGTH)) &&
			  ((available <= dataLengthIndex) || (length == (dataLengthIndex + 1 + message[dataLengthIndex])));
		}
		return plausible;
	}

	/// @brief Checks if a complete message with a valid CRC starts at `message`
	/// @param[in] message The candidate start, at least HEADER_LENGTH bytes
	/// @param[in] available Number of bytes available at `message`
	/// @returns `true` if a complete message with a valid CRC is there
	static bool is_complete_message(const std::uint8_t *message, std::size_t available)
	{
		const std::size_t length = message[1];
		bool complete = (length >= HEADER_LENGTH) && (available >= (length + CRC_LENGTH));

		if (complete)
		{
			std::uint32_t receivedCrc = 0;
			for (std::size_t i = 0; i < CRC_LENGTH; i++)
			{
				receivedCrc |= (static_cast<std::uint32_t>(message[length + i]) << (BITS_PER_BYTE * i));
			}
			complete = (receivedCrc == AX141150Plugin::crc32(message, length));
		}
		return complete;
	}

	/// @brief Decodes one CAN frame record (the frame after its type byte) starting at `data`
	/// @param[in] recordType The type byte of the record
	/// @param[in] data The record after its type byte
	/// @param[in] length Number of bytes available at `data`
	/// @param[out] frame The decoded frame
	/// @param[out] isRemoteFrame Set if the record is a remote frame
	/// @returns The number of bytes consumed, or 0 if the record is malformed
	static std::size_t decode_frame_record(std::uint8_t recordType, const std::uint8_t *data, std::size_t length, CANMessageFrame &frame, bool &isRemoteFrame)
	{
		const std::size_t identifierLength = get_identifier_length(recordType);
		std::size_t consumed = 0;

		if ((0 != identifierLength) && (length > identifierLength))
		{
			const std::uint8_t dataLength = data[identifierLength];

			if ((dataLength <= CAN_DATA_LENGTH) && (length >= (identifierLength + 1 + dataLength)))
			{
				std::uint32_t identifier = 0;
				for (std::size_t i = 0; i < identifierLength; i++)
				{
					identifier |= (static_cast<std::uint32_t>(data[i]) << (BITS_PER_BYTE * i));
				}

				if (EXTENDED_IDENTIFIER_LENGTH == identifierLength)
				{
					isRemoteFrame = (0 != (identifier & EXTENDED_RTR_FLAG));
					frame.identifier = identifier & EXTENDED_IDENTIFIER_MASK;
					frame.isExtendedFrame = true;
				}
				else
				{
					isRemoteFrame = (0 != (identifier & STANDARD_RTR_FLAG));
					frame.identifier = identifier & STANDARD_IDENTIFIER_MASK;
					frame.isExtendedFrame = false;
				}
				frame.channel = 0;
				frame.dataLength = dataLength;
				std::memset(frame.data, 0, sizeof(frame.data));
				std::memcpy(frame.data, data + identifierLength + 1, dataLength);
				frame.timestamp_us = SystemTiming::get_timestamp_us();
				consumed = identifierLength + 1 + dataLength;
			}
		}
		return consumed;
	}

	/// @brief Formats bytes as hex for logging
	/// @param[in] data The bytes to format
	/// @returns The bytes as space separated hex
	static std::string to_hex(const std::vector<std::uint8_t> &data)
	{
		static const char DIGITS[] = "0123456789ABCDEF";
		std::string text;
		for (std::uint8_t byte : data)
		{
			if (!text.empty())
			{
				text += ' ';
			}
			text += DIGITS[byte >> BITS_PER_NIBBLE];
			text += DIGITS[byte & NIBBLE_MASK];
		}
		return text;
	}

	AX141150Plugin::AX141150Plugin(const std::string &portOrAddress, Mode mode, const std::string &pin) :
	  portOrAddress(portOrAddress),
	  pin(DEFAULT_PIN),
	  mode(mode)
	{
		if (!set_pin(pin))
		{
			LOG_ERROR("[AX141150] The PIN must be exactly 6 digits, using the default PIN " + std::string(DEFAULT_PIN) + " instead.");
		}
	}

	AX141150Plugin::~AX141150Plugin()
	{
		close();
	}

	std::string AX141150Plugin::get_name() const
	{
		return "Axiomatic AX141150";
	}

	bool AX141150Plugin::get_is_valid() const
	{
		LOCK_GUARD(Mutex, connectionMutex);
#ifdef _WIN32
		return (nullptr != handle);
#else
		return (-1 != fileDescriptor);
#endif
	}

	void AX141150Plugin::close()
	{
		LOCK_GUARD(Mutex, connectionMutex);
#ifdef _WIN32
		if (nullptr != handle)
		{
			CloseHandle(static_cast<HANDLE>(handle));
			handle = nullptr;
		}
#else
		if (-1 != fileDescriptor)
		{
			::close(fileDescriptor);
			fileDescriptor = -1;
		}
#endif
	}

	void AX141150Plugin::open()
	{
		close();
		receiveBuffer.clear();
		receivedFrames.clear();
		{
			LOCK_GUARD(Mutex, settingsMutex);
			lastError.clear();
			lastConfigurationResponse.clear();
			configurationResponseCount = 0;
		}

		bool opened = false;
		if (is_bluetooth_address(portOrAddress))
		{
#ifdef __linux__
			opened = open_rfcomm_socket();
#else
			fail_open("connecting by Bluetooth address is only supported on Linux. Pair the device and use its serial port instead.");
#endif
		}
		else
		{
			opened = open_serial_port();
		}

		if (opened)
		{
			if (apply_configuration(true))
			{
				LOG_INFO("[AX141150] Connected to " + portOrAddress + " in " + std::string(Mode::Bridge == get_mode() ? "bridge" : "interface") + " mode.");
			}
			else
			{
				fail_open("could not send the configuration to the converter.");
			}
		}
	}

	bool AX141150Plugin::read_frame(isobus::CANMessageFrame &canFrame)
	{
		if (receivedFrames.empty() && get_is_valid())
		{
			receive_and_decode(RECEIVE_TIMEOUT_MS);
		}

		if (receivedFrames.empty())
		{
			return false;
		}

		canFrame = receivedFrames.front();
		receivedFrames.pop_front();
		return true;
	}

	bool AX141150Plugin::write_frame(const isobus::CANMessageFrame &canFrame)
	{
		const std::vector<std::uint8_t> message = encode_can_frame(canFrame);
		return (!message.empty()) && write_bytes(message);
	}

	std::string AX141150Plugin::get_last_error() const
	{
		LOCK_GUARD(Mutex, settingsMutex);
		return lastError;
	}

	bool AX141150Plugin::set_pin(const std::string &newPin)
	{
		if (!is_valid_pin(newPin))
		{
			return false;
		}
		LOCK_GUARD(Mutex, settingsMutex);
		pin = newPin;
		return true;
	}

	std::string AX141150Plugin::get_pin() const
	{
		LOCK_GUARD(Mutex, settingsMutex);
		return pin;
	}

	bool AX141150Plugin::set_mode(Mode newMode)
	{
		{
			LOCK_GUARD(Mutex, settingsMutex);
			mode = newMode;
		}

		if (get_is_valid())
		{
			// The receive thread may be reading, so don't wait for acknowledgements here.
			// They are decoded by read_frame() and available from get_last_configuration_response().
			return apply_configuration(false);
		}
		return true;
	}

	AX141150Plugin::Mode AX141150Plugin::get_mode() const
	{
		LOCK_GUARD(Mutex, settingsMutex);
		return mode;
	}

	bool AX141150Plugin::set_baud_rate_index(std::uint8_t index)
	{
		LOCK_GUARD(Mutex, settingsMutex);
		baudRateIndex = index;
		return true;
	}

	std::uint8_t AX141150Plugin::get_baud_rate_index() const
	{
		LOCK_GUARD(Mutex, settingsMutex);
		return baudRateIndex;
	}

	bool AX141150Plugin::set_rfcomm_channel(std::uint8_t channel)
	{
		if (get_is_valid() || (channel < MIN_RFCOMM_CHANNEL) || (channel > MAX_RFCOMM_CHANNEL))
		{
			return false;
		}
		rfcommChannel = channel;
		return true;
	}

	bool AX141150Plugin::change_device_pin(PinType type, const std::string &oldPin, const std::string &newPin)
	{
		if (!get_is_valid())
		{
			set_last_error("Cannot change a PIN on the converter while the driver is closed.");
			return false;
		}
		if (!is_valid_pin(oldPin) || !is_valid_pin(newPin))
		{
			set_last_error("PINs must be exactly 6 digits.");
			return false;
		}

		std::vector<std::uint8_t> payload;
		payload.reserve(1 + (2 * PIN_LENGTH));
		payload.push_back(static_cast<std::uint8_t>(type));
		payload.insert(payload.end(), oldPin.begin(), oldPin.end());
		payload.insert(payload.end(), newPin.begin(), newPin.end());

		bool success = send_configuration_command(build_enter_configuration_payload(), false) &&
		  send_configuration_command(payload, false);

		if (success && (PinType::Configuration == type))
		{
			set_pin(newPin);
		}

		success = send_configuration_command({ static_cast<std::uint8_t>(ConfigurationCommand::ExitConfigurationMode) }, false) && success;
		return success;
	}

	std::vector<std::uint8_t> AX141150Plugin::get_last_configuration_response() const
	{
		LOCK_GUARD(Mutex, settingsMutex);
		return lastConfigurationResponse;
	}

	bool AX141150Plugin::is_valid_pin(const std::string &pinToCheck)
	{
		return (PIN_LENGTH == pinToCheck.size()) &&
		  std::all_of(pinToCheck.begin(), pinToCheck.end(), [](char c) { return (c >= '0') && (c <= '9'); });
	}

	bool AX141150Plugin::is_bluetooth_address(const std::string &text)
	{
		if (BLUETOOTH_ADDRESS_TEXT_LENGTH != text.size())
		{
			return false;
		}
		for (std::size_t i = 0; i < text.size(); i++)
		{
			if ((BLUETOOTH_ADDRESS_TEXT_STRIDE - 1) == (i % BLUETOOTH_ADDRESS_TEXT_STRIDE))
			{
				if (':' != text[i])
				{
					return false;
				}
			}
			else if (0 == std::isxdigit(static_cast<unsigned char>(text[i])))
			{
				return false;
			}
		}
		return true;
	}

	std::uint32_t AX141150Plugin::crc32(const std::uint8_t *data, std::size_t length)
	{
		std::uint32_t crc = CRC32_INITIAL_VALUE;
		for (std::size_t i = 0; i < length; i++)
		{
			crc ^= data[i];
			for (std::uint32_t bit = 0; bit < BITS_PER_BYTE; bit++)
			{
				crc = (crc >> 1) ^ (CRC32_POLYNOMIAL & (0 - (crc & 1)));
			}
		}
		return ~crc;
	}

	std::vector<std::uint8_t> AX141150Plugin::encode_message(std::uint8_t type, const std::vector<std::uint8_t> &payload)
	{
		std::vector<std::uint8_t> message;

		if ((HEADER_LENGTH + payload.size()) > MAX_MESSAGE_LENGTH)
		{
			return message;
		}

		message.reserve(HEADER_LENGTH + payload.size() + CRC_LENGTH);
		message.push_back(type);
		message.push_back(static_cast<std::uint8_t>(HEADER_LENGTH + payload.size()));
		message.insert(message.end(), payload.begin(), payload.end());

		const std::uint32_t crc = crc32(message.data(), message.size());
		for (std::size_t i = 0; i < CRC_LENGTH; i++)
		{
			message.push_back(static_cast<std::uint8_t>(crc >> (BITS_PER_BYTE * i)));
		}
		return message;
	}

	std::vector<std::uint8_t> AX141150Plugin::encode_can_frame(const CANMessageFrame &canFrame)
	{
		std::vector<std::uint8_t> message;

		if (canFrame.dataLength <= CAN_DATA_LENGTH)
		{
			const bool extended = canFrame.isExtendedFrame;
			const std::uint32_t identifier = canFrame.identifier & (extended ? EXTENDED_IDENTIFIER_MASK : STANDARD_IDENTIFIER_MASK);
			const std::size_t identifierLength = extended ? EXTENDED_IDENTIFIER_LENGTH : STANDARD_IDENTIFIER_LENGTH;
			const MessageType type = extended ? MessageType::CANDataExtendedToDevice : MessageType::CANDataStandardToDevice;
			std::vector<std::uint8_t> payload;

			for (std::size_t i = 0; i < identifierLength; i++)
			{
				payload.push_back(static_cast<std::uint8_t>(identifier >> (BITS_PER_BYTE * i)));
			}
			payload.push_back(canFrame.dataLength);
			payload.insert(payload.end(), canFrame.data, canFrame.data + canFrame.dataLength);
			message = encode_message(static_cast<std::uint8_t>(type), payload);
		}
		return message;
	}

	void AX141150Plugin::decode_messages(std::vector<std::uint8_t> &buffer,
	                                     std::deque<CANMessageFrame> &frames,
	                                     std::vector<std::vector<std::uint8_t>> &configurationResponses)
	{
		std::size_t position = 0;

		while ((buffer.size() - position) >= (HEADER_LENGTH + CRC_LENGTH))
		{
			const std::uint8_t type = buffer[position];
			const std::size_t length = buffer[position + 1];

			if ((!is_known_device_message_type(type)) || (!is_plausible_length(buffer.data() + position, buffer.size() - position)))
			{
				position++; // Not the start of a message, resynchronise
				continue;
			}

			if ((buffer.size() - position) < (length + CRC_LENGTH))
			{
				// Either the rest of the message hasn't arrived yet, or this is a false start.
				// If a complete valid message follows, it's a false start, so skip it instead of waiting.
				bool laterMessageFound = false;
				for (std::size_t candidate = position + 1; (!laterMessageFound) && ((buffer.size() - candidate) >= (HEADER_LENGTH + CRC_LENGTH)); candidate++)
				{
					laterMessageFound = is_known_device_message_type(buffer[candidate]) &&
					  is_plausible_length(buffer.data() + candidate, buffer.size() - candidate) &&
					  is_complete_message(buffer.data() + candidate, buffer.size() - candidate);
				}

				if (laterMessageFound)
				{
					position++;
					continue;
				}
				break; // Wait for the rest of the message
			}

			const std::uint8_t *message = buffer.data() + position;

			if (!is_complete_message(message, buffer.size() - position))
			{
				position++; // Corrupted or false start, resynchronise
				continue;
			}

			const std::uint8_t *payload = message + HEADER_LENGTH;
			const std::size_t payloadLength = length - HEADER_LENGTH;

			switch (type)
			{
				case static_cast<std::uint8_t>(MessageType::CANDataStandardFromDevice):
				case static_cast<std::uint8_t>(MessageType::CANDataExtendedFromDevice):
				{
					CANMessageFrame frame;
					bool isRemoteFrame = false;
					if ((0 != decode_frame_record(type, payload, payloadLength, frame, isRemoteFrame)) && (!isRemoteFrame))
					{
						frames.push_back(frame);
					}
				}
				break;

				case static_cast<std::uint8_t>(MessageType::MultipleCANFramesFromDevice):
				{
					std::size_t offset = 0;
					while (offset < payloadLength)
					{
						CANMessageFrame frame;
						bool isRemoteFrame = false;
						const std::size_t consumed = decode_frame_record(payload[offset], payload + offset + 1, payloadLength - offset - 1, frame, isRemoteFrame);
						if (0 == consumed)
						{
							break;
						}
						if (!isRemoteFrame)
						{
							frames.push_back(frame);
						}
						offset += consumed + 1;
					}
				}
				break;

				case static_cast<std::uint8_t>(MessageType::ChangeConfigurationResponse):
				{
					configurationResponses.emplace_back(payload, payload + payloadLength);
				}
				break;

				default:
				{
					// Valid message that this driver doesn't use (MAP responses, LED state, ...)
				}
				break;
			}
			position += length + CRC_LENGTH;
		}

		buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(position));
	}

	bool AX141150Plugin::open_serial_port()
	{
#ifdef _WIN32
		std::string path = portOrAddress;
		if (0 != path.compare(0, 4, "\\\\.\\"))
		{
			path = "\\\\.\\" + path; // Required for COM10 and above
		}

		HANDLE portHandle = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
		if (INVALID_HANDLE_VALUE == portHandle)
		{
			fail_open("CreateFile failed with error " + isobus::to_string(GetLastError()) + ".");
			return false;
		}
		{
			LOCK_GUARD(Mutex, connectionMutex);
			handle = portHandle;
		}

		DCB settings;
		std::memset(&settings, 0, sizeof(settings));
		settings.DCBlength = sizeof(settings);
		if (!GetCommState(portHandle, &settings))
		{
			fail_open("GetCommState failed with error " + isobus::to_string(GetLastError()) + ".");
			return false;
		}
		settings.BaudRate = CBR_115200; // Ignored by Bluetooth virtual COM ports, but must be valid
		settings.ByteSize = 8;
		settings.Parity = NOPARITY;
		settings.StopBits = ONESTOPBIT;
		settings.fBinary = TRUE;
		settings.fParity = FALSE;
		settings.fOutxCtsFlow = FALSE;
		settings.fOutxDsrFlow = FALSE;
		settings.fDtrControl = DTR_CONTROL_ENABLE;
		settings.fRtsControl = RTS_CONTROL_ENABLE;
		settings.fOutX = FALSE;
		settings.fInX = FALSE;
		if (!SetCommState(portHandle, &settings))
		{
			fail_open("SetCommState failed with error " + isobus::to_string(GetLastError()) + ".");
			return false;
		}

		// Return from ReadFile as soon as any byte arrives, or after RECEIVE_TIMEOUT_MS with nothing
		COMMTIMEOUTS timeouts;
		std::memset(&timeouts, 0, sizeof(timeouts));
		timeouts.ReadIntervalTimeout = MAXDWORD;
		timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
		timeouts.ReadTotalTimeoutConstant = RECEIVE_TIMEOUT_MS;
		timeouts.WriteTotalTimeoutConstant = WRITE_TIMEOUT_MS;
		if (!SetCommTimeouts(portHandle, &timeouts))
		{
			fail_open("SetCommTimeouts failed with error " + isobus::to_string(GetLastError()) + ".");
			return false;
		}
		PurgeComm(portHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);
		return true;
#else
		{
			LOCK_GUARD(Mutex, connectionMutex);
			fileDescriptor = ::open(portOrAddress.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
		}
		if (-1 == fileDescriptor)
		{
			fail_open("open() failed: " + std::string(std::strerror(errno)));
			return false;
		}

		struct termios settings;
		if (0 == tcgetattr(fileDescriptor, &settings))
		{
			cfmakeraw(&settings);
			cfsetispeed(&settings, B115200); // Ignored by RFCOMM TTYs, but must be valid
			cfsetospeed(&settings, B115200);
			settings.c_cflag |= (CLOCAL | CREAD);
			settings.c_cc[VMIN] = 0;
			settings.c_cc[VTIME] = 0;
			if (0 != tcsetattr(fileDescriptor, TCSANOW, &settings))
			{
				fail_open("tcsetattr() failed: " + std::string(std::strerror(errno)));
				return false;
			}
			tcflush(fileDescriptor, TCIOFLUSH);
		}
		// else: not a TTY (for example a pipe used in testing), use it as a plain byte stream

		// Blocking writes, reads are guarded by poll()
		const int flags = fcntl(fileDescriptor, F_GETFL, 0);
		if ((-1 != flags) && (-1 == fcntl(fileDescriptor, F_SETFL, flags & ~O_NONBLOCK)))
		{
			fail_open("fcntl() failed: " + std::string(std::strerror(errno)));
			return false;
		}
		return true;
#endif
	}

#ifdef __linux__
	bool AX141150Plugin::open_rfcomm_socket()
	{
		// Mirrors struct sockaddr_rc from BlueZ, so there's no dependency on libbluetooth headers
		struct RfcommSocketAddress
		{
			sa_family_t family;
			std::uint8_t address[BLUETOOTH_ADDRESS_LENGTH];
			std::uint8_t channel;
		};
		static_assert(2 == offsetof(RfcommSocketAddress, address), "Unexpected sockaddr_rc layout");
		static_assert(8 == offsetof(RfcommSocketAddress, channel), "Unexpected sockaddr_rc layout");

		constexpr int ADDRESS_FAMILY_BLUETOOTH = 31; // AF_BLUETOOTH
		constexpr int PROTOCOL_RFCOMM = 3; // BTPROTO_RFCOMM

		RfcommSocketAddress socketAddress;
		std::memset(&socketAddress, 0, sizeof(socketAddress));
		socketAddress.family = ADDRESS_FAMILY_BLUETOOTH;
		socketAddress.channel = rfcommChannel;
		for (std::size_t i = 0; i < BLUETOOTH_ADDRESS_LENGTH; i++)
		{
			// BlueZ stores the address least significant byte first, the text form is most significant first
			socketAddress.address[BLUETOOTH_ADDRESS_LENGTH - 1 - i] = static_cast<std::uint8_t>(std::stoul(portOrAddress.substr(i * BLUETOOTH_ADDRESS_TEXT_STRIDE, BLUETOOTH_ADDRESS_TEXT_STRIDE - 1), nullptr, 16));
		}

		{
			LOCK_GUARD(Mutex, connectionMutex);
			fileDescriptor = socket(ADDRESS_FAMILY_BLUETOOTH, SOCK_STREAM, PROTOCOL_RFCOMM);
		}
		if (-1 == fileDescriptor)
		{
			fail_open("socket() failed: " + std::string(std::strerror(errno)) + ". Is Bluetooth available on this system?");
			return false;
		}

		if (0 != connect(fileDescriptor, reinterpret_cast<struct sockaddr *>(&socketAddress), sizeof(socketAddress)))
		{
			fail_open("connect() on RFCOMM channel " + isobus::to_string(static_cast<int>(rfcommChannel)) + " failed: " + std::string(std::strerror(errno)) + ". Make sure the converter is powered, in range and paired.");
			return false;
		}
		return true;
	}
#endif

	bool AX141150Plugin::apply_configuration(bool waitForAcknowledgements)
	{
		Mode modeToApply;
		std::uint8_t baudRateToApply;
		{
			LOCK_GUARD(Mutex, settingsMutex);
			modeToApply = mode;
			baudRateToApply = baudRateIndex;
		}

		bool success = send_configuration_command(build_enter_configuration_payload(), waitForAcknowledgements);

		const ConfigurationCommand modeCommand = (Mode::Bridge == modeToApply) ? ConfigurationCommand::UseBridgeMode : ConfigurationCommand::UseInterfaceMode;
		success = success && send_configuration_command({ static_cast<std::uint8_t>(modeCommand) }, waitForAcknowledgements);

		if (success && (BAUD_RATE_UNCHANGED != baudRateToApply))
		{
			success = send_configuration_command({ static_cast<std::uint8_t>(ConfigurationCommand::SetCANBaudRate), baudRateToApply }, waitForAcknowledgements);
			if (success)
			{
				LOG_WARNING("[AX141150] CAN baud rate index " + isobus::to_string(static_cast<int>(baudRateToApply)) + " written. Power cycle the converter for it to take effect.");
			}
		}

		// Always try to leave configuration mode, even if something above failed
		success = send_configuration_command({ static_cast<std::uint8_t>(ConfigurationCommand::ExitConfigurationMode) }, waitForAcknowledgements) && success;
		return success;
	}

	bool AX141150Plugin::send_configuration_command(const std::vector<std::uint8_t> &payload, bool waitForAcknowledgement)
	{
		std::size_t responsesBefore;
		{
			LOCK_GUARD(Mutex, settingsMutex);
			responsesBefore = configurationResponseCount;
		}

		if (!write_bytes(encode_message(static_cast<std::uint8_t>(MessageType::ChangeConfiguration), payload)))
		{
			return false;
		}

		if (waitForAcknowledgement)
		{
			const std::uint32_t startTime = SystemTiming::get_timestamp_ms();
			bool acknowledged = false;

			while ((!acknowledged) && get_is_valid() && (!SystemTiming::time_expired_ms(startTime, DEFAULT_CONFIGURATION_TIMEOUT_MS)))
			{
				receive_and_decode(ACKNOWLEDGEMENT_POLL_INTERVAL_MS);
				LOCK_GUARD(Mutex, settingsMutex);
				acknowledged = (configurationResponseCount > responsesBefore);
			}

			if (!acknowledged)
			{
				LOG_WARNING("[AX141150] No acknowledgement for configuration command 0x" + to_hex({ payload.front() }) + ". Check the PIN if the mode doesn't change.");
			}
		}
		return get_is_valid();
	}

	std::vector<std::uint8_t> AX141150Plugin::build_enter_configuration_payload() const
	{
		std::vector<std::uint8_t> payload;
		payload.push_back(static_cast<std::uint8_t>(ConfigurationCommand::EnterConfigurationMode));

		// PIN digits are sent as ASCII characters, one per byte ("000000" -> 0x30 x 6), as in Axiomatic's AN404 example
		const std::string currentPin = get_pin();
		payload.insert(payload.end(), currentPin.begin(), currentPin.end());
		return payload;
	}

	bool AX141150Plugin::write_bytes(const std::vector<std::uint8_t> &data)
	{
		LOCK_GUARD(Mutex, connectionMutex);
#ifdef _WIN32
		DWORD written = 0;
		return (!data.empty()) &&
		  (nullptr != handle) &&
		  (TRUE == WriteFile(static_cast<HANDLE>(handle), data.data(), static_cast<DWORD>(data.size()), &written, nullptr)) &&
		  (written == data.size());
#else
		bool success = (!data.empty()) && (-1 != fileDescriptor);
		std::size_t total = 0;
		while (success && (total < data.size()))
		{
			const ssize_t result = ::write(fileDescriptor, data.data() + total, data.size() - total);
			if (result >= 0)
			{
				total += static_cast<std::size_t>(result);
			}
			else if (EINTR != errno)
			{
				set_last_error("write() failed: " + std::string(std::strerror(errno)));
				success = false;
			}
		}
		return success;
#endif
	}

	bool AX141150Plugin::receive_and_decode(std::uint32_t timeoutMs)
	{
		std::uint8_t chunk[READ_CHUNK_SIZE];
		std::size_t bytesRead = 0;

#ifdef _WIN32
		(void)timeoutMs; // The read timeout is fixed by SetCommTimeouts in open_serial_port()
		DWORD count = 0;
		if (!ReadFile(static_cast<HANDLE>(handle), chunk, sizeof(chunk), &count, nullptr))
		{
			set_last_error("ReadFile failed with error " + isobus::to_string(GetLastError()) + ", closing the connection.");
			close();
			return false;
		}
		bytesRead = count;
#else
		struct pollfd pollingFileDescriptor;
		pollingFileDescriptor.fd = fileDescriptor;
		pollingFileDescriptor.events = POLLIN;
		pollingFileDescriptor.revents = 0;

		const int pollResult = poll(&pollingFileDescriptor, 1, static_cast<int>(timeoutMs));
		if (pollResult <= 0)
		{
			return false;
		}

		if (0 != (pollingFileDescriptor.revents & POLLIN))
		{
			const ssize_t result = ::read(fileDescriptor, chunk, sizeof(chunk));
			if (result > 0)
			{
				bytesRead = static_cast<std::size_t>(result);
			}
			else if ((0 == result) || ((EINTR != errno) && (EAGAIN != errno)))
			{
				set_last_error("The connection to the converter was lost, closing it.");
				close();
				return false;
			}
		}
		else if (0 != (pollingFileDescriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
		{
			set_last_error("The connection to the converter was lost, closing it.");
			close();
			return false;
		}
#endif

		if (0 == bytesRead)
		{
			return false;
		}

		receiveBuffer.insert(receiveBuffer.end(), chunk, chunk + bytesRead);

		std::vector<std::vector<std::uint8_t>> responses;
		decode_messages(receiveBuffer, receivedFrames, responses);

		if (receiveBuffer.size() > MAX_RECEIVE_BUFFER_SIZE)
		{
			receiveBuffer.clear(); // Shouldn't happen since the decoder always makes progress, but don't grow forever
		}

		if (!responses.empty())
		{
			for (const auto &response : responses)
			{
				LOG_DEBUG("[AX141150] Configuration response: " + to_hex(response));
			}
			LOCK_GUARD(Mutex, settingsMutex);
			lastConfigurationResponse = responses.back();
			configurationResponseCount += responses.size();
		}
		return true;
	}

	void AX141150Plugin::set_last_error(const std::string &error)
	{
		{
			LOCK_GUARD(Mutex, settingsMutex);
			lastError = error;
		}
		LOG_ERROR("[AX141150] " + error);
	}

	void AX141150Plugin::fail_open(const std::string &reason)
	{
		close();
		set_last_error("Unable to open " + portOrAddress + ", " + reason);
	}
}
