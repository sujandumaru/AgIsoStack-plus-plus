//================================================================================================
/// @file axiomatic_ax141150_plugin.hpp
///
/// @brief A CAN driver for the Axiomatic AX141150 CAN-Bluetooth converter (CAN2BT).
/// @details The AX141150 exposes the CAN bus over Bluetooth Serial Port Profile (SPP), using
/// Axiomatic's proprietary binary SPP message format:
///
/// `<msg type> <length> <payload 0> ... <payload n-1> <crc32 b0> <crc32 b1> <crc32 b2> <crc32 b3>`
///
/// where `<length>` counts the type byte, the length byte and the payload (not the CRC), and the
/// CRC32 is the standard (zlib / java.util.zip) CRC32 of those bytes, sent least significant byte first.
///
/// The driver can reach the converter two ways:
/// - Through a serial port that the operating system created for the paired device, for example
///   `/dev/rfcomm0` on Linux, `/dev/tty.CAN2BT` on macOS or `COM5` on Windows.
/// - On Linux only, directly over an RFCOMM socket by passing the converter's Bluetooth
///   address, for example `00:11:22:33:44:55`. The device must already be paired.
///
/// When the driver opens, it enters the converter's configuration mode with the configured PIN,
/// applies the selected operating mode (and baud rate, if one was set), then leaves configuration mode.
///
/// @author Open-Agriculture contributors
///
/// @copyright 2026 The Open-Agriculture Developers
//================================================================================================
#ifndef AXIOMATIC_AX141150_PLUGIN_HPP
#define AXIOMATIC_AX141150_PLUGIN_HPP

#include "isobus/hardware_integration/can_hardware_plugin.hpp"
#include "isobus/isobus/can_hardware_abstraction.hpp"
#include "isobus/isobus/can_message_frame.hpp"
#include "isobus/utility/thread_synchronization.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace isobus
{
	//================================================================================================
	/// @class AX141150Plugin
	///
	/// @brief A CAN driver for the Axiomatic AX141150 CAN-Bluetooth converter.
	//================================================================================================
	class AX141150Plugin : public CANHardwarePlugin
	{
	public:
		/// @brief The operating modes of the AX141150
		enum class Mode : std::uint8_t
		{
			Bridge, ///< CAN receive filtering disabled, every frame on the bus is forwarded (protocol independent). The converter is invisible on the bus. Recommended for use with this stack.
			Interface ///< The converter's default SAE J1939 receive filtering is active and it takes part in address claiming. Only frames that pass its receive filters are forwarded.
		};

		/// @brief The PINs stored on the AX141150 that can be changed with change_device_pin()
		enum class PinType : std::uint8_t
		{
			Pairing = 0x20, ///< The PIN used when pairing the converter with a host
			Configuration = 0x21, ///< The PIN used to enter configuration mode (the one passed to set_pin())
			RemoteAccess = 0x22 ///< The PIN used by a remote CAN2BT node in bridge-pair setups
		};

		/// @brief The SPP message types this driver sends or recognises
		enum class MessageType : std::uint8_t
		{
			CANDataStandardToDevice = 0x12, ///< Host -> CAN2BT, 11-bit identifier frame
			CANDataExtendedToDevice = 0x13, ///< Host -> CAN2BT, 29-bit identifier frame
			CANDataStandardFromDevice = 0x21, ///< CAN2BT -> host, 11-bit identifier frame
			CANDataExtendedFromDevice = 0x31, ///< CAN2BT -> host, 29-bit identifier frame
			MapWriteResponse = 0x43, ///< CAN2BT -> host, MAP write response (ignored)
			MapReadResponse = 0x54, ///< CAN2BT -> host, MAP read response (ignored)
			DownloadFlashResponse = 0x65, ///< CAN2BT -> host, download flash contents response (ignored)
			ChangeConfiguration = 0x67, ///< Host -> CAN2BT, configuration command
			DownloadFlashAcknowledgeResponse = 0x75, ///< CAN2BT -> host, download flash contents ACK response (ignored)
			ChangeConfigurationResponse = 0x76, ///< CAN2BT -> host, acknowledgement of a configuration command
			LEDStateResponse = 0x87, ///< CAN2BT -> host, LED state response (ignored)
			RequestPGNResponse = 0x98, ///< CAN2BT -> host, POLL/request PGN response (ignored)
			MultipleCANFramesFromDevice = 0xFE ///< CAN2BT -> host, up to 6 frames in one message (firmware v3.00+)
		};

		/// @brief The configuration commands (first payload byte of a ChangeConfiguration message) used by this driver
		enum class ConfigurationCommand : std::uint8_t
		{
			UseBridgeMode = 0x03, ///< Disable CAN receive filtering
			UseInterfaceMode = 0x04, ///< Enable the default CAN receive filtering
			SetCANBaudRate = 0x05, ///< Set the CAN baud rate index (takes effect after a power cycle)
			SetTxEcho = 0x06, ///< Echo frames sent from the host back to the host
			EnterConfigurationMode = 0xC0, ///< Followed by the 6 PIN digits
			ExitConfigurationMode = 0xC1 ///< Leave configuration mode
		};

		static constexpr const char *DEFAULT_PIN = "000000"; ///< The factory default for all PINs on the AX141150
		static constexpr std::size_t PIN_LENGTH = 6; ///< PINs on the AX141150 are always 6 digits
		static constexpr std::uint8_t DEFAULT_RFCOMM_CHANNEL = 1; ///< The RFCOMM channel used for SPP when connecting by Bluetooth address
		static constexpr std::uint8_t BAUD_RATE_UNCHANGED = 0xFF; ///< Baud rate index value meaning "don't change the baud rate on open"
		static constexpr std::uint32_t DEFAULT_CONFIGURATION_TIMEOUT_MS = 500; ///< How long open() waits for each configuration acknowledgement

		/// @brief Constructor for the AX141150 driver
		/// @param[in] portOrAddress A serial port (`/dev/rfcomm0`, `COM5`, ...) or, on Linux, a Bluetooth address (`00:11:22:33:44:55`)
		/// @param[in] mode The operating mode to put the converter in when the driver opens
		/// @param[in] pin The 6 digit configuration PIN of the converter, used to enter configuration mode
		explicit AX141150Plugin(const std::string &portOrAddress, Mode mode = Mode::Bridge, const std::string &pin = DEFAULT_PIN);

		/// @brief The destructor for AX141150Plugin
		virtual ~AX141150Plugin();

		/// @brief Returns the displayable name of the plugin
		/// @returns "Axiomatic AX141150"
		std::string get_name() const override;

		/// @brief Returns if the connection to the converter is open
		/// @returns `true` if connected, `false` if not connected
		bool get_is_valid() const override;

		/// @brief Closes the connection to the converter
		void close() override;

		/// @brief Opens the connection to the converter and applies the configured PIN, mode and baud rate
		void open() override;

		/// @brief Returns a frame from the hardware (synchronous), or `false` if no frame can be read.
		/// @param[in, out] canFrame The CAN frame that was read
		/// @returns `true` if a CAN frame was read, otherwise `false`
		bool read_frame(isobus::CANMessageFrame &canFrame) override;

		/// @brief Writes a frame to the bus (synchronous)
		/// @param[in] canFrame The frame to write to the bus
		/// @returns `true` if the frame was written, otherwise `false`
		bool write_frame(const isobus::CANMessageFrame &canFrame) override;

		/// @brief Returns the last error that occurred, or an empty string
		/// @returns A description of the last error
		std::string get_last_error() const override;

		/// @brief Sets the configuration PIN the driver uses to enter configuration mode.
		/// @note This does not change the PIN stored on the converter, use change_device_pin() for that.
		/// @param[in] pin A 6 digit PIN, for example "000000"
		/// @returns `true` if the PIN was valid and stored, otherwise `false`
		bool set_pin(const std::string &pin);

		/// @brief Returns the configuration PIN the driver uses to enter configuration mode
		/// @returns The configured PIN
		std::string get_pin() const;

		/// @brief Sets the operating mode. If the driver is open, the mode is applied to the converter right away,
		/// otherwise it is applied the next time the driver opens.
		/// @param[in] mode The mode to use
		/// @returns `true` if the mode was stored (and, if open, the commands were sent), otherwise `false`
		bool set_mode(Mode mode);

		/// @brief Returns the operating mode the driver applies to the converter
		/// @returns The configured mode
		Mode get_mode() const;

		/// @brief Sets the CAN baud rate index to write to the converter when the driver opens.
		/// @details The AX141150 user manual lists the index as 1 = 10k, 2 = 20k, 3 = 50k, 4 = 100k, 5 = 125k,
		/// 6 = 250k, 7 = 500k, 9 = 1M, but also repeats the older AX141100 table (0 = 50k ... 3 = 250k ... 5 = 1M).
		/// Check the table for your firmware before using this. The converter must be power cycled before a new
		/// baud rate takes effect. The default (BAUD_RATE_UNCHANGED) leaves the converter's baud rate alone.
		/// @param[in] index The raw baud rate index, or BAUD_RATE_UNCHANGED
		/// @returns `true` if the index was stored
		bool set_baud_rate_index(std::uint8_t index);

		/// @brief Returns the configured baud rate index
		/// @returns The baud rate index, or BAUD_RATE_UNCHANGED
		std::uint8_t get_baud_rate_index() const;

		/// @brief Sets the RFCOMM channel used when connecting directly by Bluetooth address (Linux only)
		/// @param[in] channel The RFCOMM channel, 1 to 30
		/// @returns `true` if the channel was stored, `false` if the driver is open or the channel is out of range
		bool set_rfcomm_channel(std::uint8_t channel);

		/// @brief Changes one of the PINs stored on the converter. The driver must be open.
		/// @details If the configuration PIN is changed, the driver also starts using the new PIN.
		/// @param[in] type Which PIN to change
		/// @param[in] oldPin The current 6 digit PIN stored on the converter
		/// @param[in] newPin The new 6 digit PIN
		/// @returns `true` if the commands were sent, otherwise `false`
		bool change_device_pin(PinType type, const std::string &oldPin, const std::string &newPin);

		/// @brief Returns the payload of the most recent configuration acknowledgement (0x76) received
		/// @returns The raw acknowledgement payload, empty if none has been received
		std::vector<std::uint8_t> get_last_configuration_response() const;

		/// @brief Checks if a string is a valid 6 digit AX141150 PIN
		/// @param[in] pin The PIN to check
		/// @returns `true` if the PIN is exactly 6 decimal digits
		static bool is_valid_pin(const std::string &pin);

		/// @brief Checks if a string is a Bluetooth address in the form XX:XX:XX:XX:XX:XX
		/// @param[in] text The string to check
		/// @returns `true` if the string is a Bluetooth address
		static bool is_bluetooth_address(const std::string &text);

		/// @brief Computes the CRC32 used by the SPP message format (the standard zlib CRC32)
		/// @param[in] data The bytes to checksum
		/// @param[in] length The number of bytes
		/// @returns The CRC32
		static std::uint32_t crc32(const std::uint8_t *data, std::size_t length);

		/// @brief Builds a complete SPP message, including length and CRC
		/// @param[in] type The message type
		/// @param[in] payload The message payload (at most 253 bytes)
		/// @returns The encoded message, or an empty vector if the payload is too long
		static std::vector<std::uint8_t> encode_message(std::uint8_t type, const std::vector<std::uint8_t> &payload);

		/// @brief Builds the SPP message that sends a CAN frame to the bus
		/// @param[in] canFrame The frame to encode
		/// @returns The encoded message, or an empty vector if the frame is invalid
		static std::vector<std::uint8_t> encode_can_frame(const CANMessageFrame &canFrame);

		/// @brief Decodes every complete SPP message at the start of a receive buffer.
		/// @details Consumed bytes are removed from the buffer. Bytes that don't start a valid message are skipped
		/// one at a time so the decoder resynchronises after corruption. An incomplete message is left in the buffer.
		/// @param[in, out] buffer The receive buffer
		/// @param[out] frames Decoded CAN frames are appended here
		/// @param[out] configurationResponses Payloads of decoded configuration acknowledgements are appended here
		static void decode_messages(std::vector<std::uint8_t> &buffer,
		                            std::deque<CANMessageFrame> &frames,
		                            std::vector<std::vector<std::uint8_t>> &configurationResponses);

	private:
		/// @brief Opens a serial port to the converter
		/// @returns `true` if successful
		bool open_serial_port();

#ifdef __linux__
		/// @brief Opens an RFCOMM socket to the converter
		/// @returns `true` if successful
		bool open_rfcomm_socket();
#endif

		/// @brief Sends the configuration sequence for the current mode and baud rate
		/// @param[in] waitForAcknowledgements Wait for each acknowledgement. Only safe when nothing else is reading.
		/// @returns `true` if all commands were sent
		bool apply_configuration(bool waitForAcknowledgements);

		/// @brief Sends one configuration command
		/// @param[in] payload The configuration payload (command byte and arguments)
		/// @param[in] waitForAcknowledgement Wait for the acknowledgement
		/// @returns `true` if the command was sent
		bool send_configuration_command(const std::vector<std::uint8_t> &payload, bool waitForAcknowledgement);

		/// @brief Builds the "enter configuration mode" payload with the configured PIN
		/// @returns The payload
		std::vector<std::uint8_t> build_enter_configuration_payload() const;

		/// @brief Writes raw bytes to the converter
		/// @param[in] data The bytes to write
		/// @returns `true` if all bytes were written
		bool write_bytes(const std::vector<std::uint8_t> &data);

		/// @brief Reads available bytes into the receive buffer and decodes them
		/// @param[in] timeoutMs How long to wait for data
		/// @returns `true` if any bytes were read
		bool receive_and_decode(std::uint32_t timeoutMs);

		/// @brief Records and logs an error
		/// @param[in] error The error description
		void set_last_error(const std::string &error);

		/// @brief Closes the connection and records why
		/// @param[in] reason The reason opening failed
		void fail_open(const std::string &reason);

		std::string portOrAddress; ///< The serial port or Bluetooth address of the converter
		std::string pin; ///< The configuration PIN
		Mode mode; ///< The operating mode to apply
		std::uint8_t baudRateIndex = BAUD_RATE_UNCHANGED; ///< The baud rate index to apply on open
		std::uint8_t rfcommChannel = DEFAULT_RFCOMM_CHANNEL; ///< The RFCOMM channel for direct connections

#ifdef _WIN32
		void *handle = nullptr; ///< The Windows serial port handle
#else
		int fileDescriptor = -1; ///< The serial port or socket file descriptor
#endif

		std::vector<std::uint8_t> receiveBuffer; ///< Bytes received but not yet decoded
		std::deque<CANMessageFrame> receivedFrames; ///< Decoded frames not yet returned by read_frame
		std::vector<std::uint8_t> lastConfigurationResponse; ///< Payload of the last acknowledgement
		std::size_t configurationResponseCount = 0; ///< Number of acknowledgements received since opening
		std::string lastError; ///< The last error

		mutable Mutex settingsMutex; ///< Protects the PIN, mode, baud rate, error and acknowledgement fields
		mutable Mutex connectionMutex; ///< Protects the handle, since the receive thread closes it on a read error while another thread may be writing
	};
}

#endif // AXIOMATIC_AX141150_PLUGIN_HPP
