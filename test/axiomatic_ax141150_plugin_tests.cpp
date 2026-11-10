//================================================================================================
/// @file axiomatic_ax141150_plugin_tests.cpp
///
/// @brief Unit tests for the Axiomatic AX141150 CAN driver.
/// @author Sujan Dumaru
///
/// @copyright 2026 The Open-Agriculture Developers
//================================================================================================
#include <gtest/gtest.h>

#include "isobus/hardware_integration/axiomatic_ax141150_plugin.hpp"

#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <atomic>
#include <mutex>
#include <thread>
#endif

using namespace isobus;

static std::vector<std::uint8_t> device_message(AX141150Plugin::MessageType type, const std::vector<std::uint8_t> &payload)
{
	return AX141150Plugin::encode_message(static_cast<std::uint8_t>(type), payload);
}

static std::vector<CANMessageFrame> decode(std::vector<std::uint8_t> &buffer, std::vector<std::vector<std::uint8_t>> *responses = nullptr)
{
	std::deque<CANMessageFrame> frames;
	std::vector<std::vector<std::uint8_t>> ignored;
	AX141150Plugin::decode_messages(buffer, frames, (nullptr != responses) ? *responses : ignored);
	return std::vector<CANMessageFrame>(frames.begin(), frames.end());
}

TEST(AX141150_PLUGIN_TESTS, Crc32MatchesZlibCheckValue)
{
	const char *check = "123456789";
	EXPECT_EQ(0xCBF43926u, AX141150Plugin::crc32(reinterpret_cast<const std::uint8_t *>(check), std::strlen(check)));
}

TEST(AX141150_PLUGIN_TESTS, EncodesExtendedFrame)
{
	CANMessageFrame frame = {};
	frame.identifier = 0x18EFFF80;
	frame.isExtendedFrame = true;
	frame.dataLength = 3;
	frame.data[0] = 0xAA;
	frame.data[1] = 0xBB;
	frame.data[2] = 0xCC;

	const std::vector<std::uint8_t> expectedStart = { 0x13, 10, 0x80, 0xFF, 0xEF, 0x18, 3, 0xAA, 0xBB, 0xCC };
	const std::vector<std::uint8_t> message = AX141150Plugin::encode_can_frame(frame);
	ASSERT_EQ(14u, message.size());
	EXPECT_TRUE(std::equal(expectedStart.begin(), expectedStart.end(), message.begin()));

	const std::uint32_t crc = AX141150Plugin::crc32(message.data(), expectedStart.size());
	EXPECT_EQ(static_cast<std::uint8_t>(crc), message[10]);
	EXPECT_EQ(static_cast<std::uint8_t>(crc >> 24), message[13]);
}

TEST(AX141150_PLUGIN_TESTS, EncodesStandardFrame)
{
	CANMessageFrame frame = {};
	frame.identifier = 0x123;
	frame.dataLength = 0;

	const std::vector<std::uint8_t> message = AX141150Plugin::encode_can_frame(frame);
	ASSERT_EQ(9u, message.size());
	EXPECT_EQ(0x12, message[0]);
	EXPECT_EQ(5, message[1]);
	EXPECT_EQ(0x23, message[2]);
	EXPECT_EQ(0x01, message[3]);
	EXPECT_EQ(0, message[4]);
}

TEST(AX141150_PLUGIN_TESTS, RejectsTooLongData)
{
	CANMessageFrame frame = {};
	frame.dataLength = 9;
	EXPECT_TRUE(AX141150Plugin::encode_can_frame(frame).empty());
	EXPECT_TRUE(AX141150Plugin::encode_message(0x67, std::vector<std::uint8_t>(254)).empty());
}

TEST(AX141150_PLUGIN_TESTS, DecodesStandardAndExtendedFrames)
{
	std::vector<std::uint8_t> buffer = device_message(AX141150Plugin::MessageType::CANDataExtendedFromDevice, { 0x80, 0xFF, 0xEF, 0x18, 2, 0x01, 0x02 });
	const std::vector<std::uint8_t> standard = device_message(AX141150Plugin::MessageType::CANDataStandardFromDevice, { 0x23, 0x01, 1, 0x55 });
	buffer.insert(buffer.end(), standard.begin(), standard.end());

	const std::vector<CANMessageFrame> frames = decode(buffer);
	ASSERT_EQ(2u, frames.size());
	EXPECT_TRUE(frames[0].isExtendedFrame);
	EXPECT_EQ(0x18EFFF80u, frames[0].identifier);
	EXPECT_EQ(2, frames[0].dataLength);
	EXPECT_EQ(0x02, frames[0].data[1]);
	EXPECT_FALSE(frames[1].isExtendedFrame);
	EXPECT_EQ(0x123u, frames[1].identifier);
	EXPECT_EQ(0x55, frames[1].data[0]);
	EXPECT_TRUE(buffer.empty());
}

TEST(AX141150_PLUGIN_TESTS, ResynchronisesAfterGarbage)
{
	std::vector<std::uint8_t> buffer = { 0x00, 0x21, 0xFF, 0x31, 0x01, 0x76 };
	const std::vector<std::uint8_t> message = device_message(AX141150Plugin::MessageType::CANDataStandardFromDevice, { 0x23, 0x01, 0 });
	buffer.insert(buffer.end(), message.begin(), message.end());

	EXPECT_EQ(1u, decode(buffer).size());
	EXPECT_TRUE(buffer.empty());
}

TEST(AX141150_PLUGIN_TESTS, WaitsForTheRestOfAMessage)
{
	const std::vector<std::uint8_t> message = device_message(AX141150Plugin::MessageType::CANDataExtendedFromDevice, { 0x80, 0xFF, 0xEF, 0x18, 8, 1, 2, 3, 4, 5, 6, 7, 8 });
	std::vector<std::uint8_t> buffer(message.begin(), message.begin() + 9);

	EXPECT_TRUE(decode(buffer).empty());
	EXPECT_EQ(9u, buffer.size());

	buffer.insert(buffer.end(), message.begin() + 9, message.end());
	const std::vector<CANMessageFrame> frames = decode(buffer);
	ASSERT_EQ(1u, frames.size());
	EXPECT_EQ(8, frames[0].data[7]);
}

TEST(AX141150_PLUGIN_TESTS, DropsMessageWithBadCrc)
{
	std::vector<std::uint8_t> buffer = device_message(AX141150Plugin::MessageType::CANDataStandardFromDevice, { 0x23, 0x01, 1, 0x55 });
	buffer.back() ^= 0xFF;

	EXPECT_TRUE(decode(buffer).empty());
}

TEST(AX141150_PLUGIN_TESTS, DropsRemoteFrames)
{
	std::vector<std::uint8_t> buffer = device_message(AX141150Plugin::MessageType::CANDataExtendedFromDevice, { 0x80, 0xFF, 0xEF, 0x98, 0 });
	const std::vector<std::uint8_t> standard = device_message(AX141150Plugin::MessageType::CANDataStandardFromDevice, { 0x23, 0x09, 0 });
	buffer.insert(buffer.end(), standard.begin(), standard.end());

	EXPECT_TRUE(decode(buffer).empty());
	EXPECT_TRUE(buffer.empty());
}

TEST(AX141150_PLUGIN_TESTS, DecodesMultipleFramesMessage)
{
	std::vector<std::uint8_t> buffer = device_message(AX141150Plugin::MessageType::MultipleCANFramesFromDevice,
	                                                  { 0x31, 0x80, 0xFF, 0xEF, 0x18, 1, 0x11, 0x21, 0x23, 0x01, 2, 0x22, 0x33 });

	const std::vector<CANMessageFrame> frames = decode(buffer);
	ASSERT_EQ(2u, frames.size());
	EXPECT_EQ(0x18EFFF80u, frames[0].identifier);
	EXPECT_EQ(0x11, frames[0].data[0]);
	EXPECT_EQ(0x123u, frames[1].identifier);
	EXPECT_EQ(0x33, frames[1].data[1]);
}

TEST(AX141150_PLUGIN_TESTS, DecodesConfigurationResponse)
{
	std::vector<std::uint8_t> buffer = device_message(AX141150Plugin::MessageType::ChangeConfigurationResponse, { 0x03, 0x00 });
	std::vector<std::vector<std::uint8_t>> responses;

	EXPECT_TRUE(decode(buffer, &responses).empty());
	ASSERT_EQ(1u, responses.size());
	EXPECT_EQ((std::vector<std::uint8_t>{ 0x03, 0x00 }), responses[0]);
}

TEST(AX141150_PLUGIN_TESTS, ValidatesPinsAndAddresses)
{
	EXPECT_TRUE(AX141150Plugin::is_valid_pin("000000"));
	EXPECT_FALSE(AX141150Plugin::is_valid_pin("12345"));
	EXPECT_FALSE(AX141150Plugin::is_valid_pin("12345a"));

	EXPECT_TRUE(AX141150Plugin::is_bluetooth_address("00:11:22:aa:BB:ff"));
	EXPECT_FALSE(AX141150Plugin::is_bluetooth_address("00-11-22-AA-BB-FF"));
	EXPECT_FALSE(AX141150Plugin::is_bluetooth_address("00:11:22:AA:BB:FG"));
	EXPECT_FALSE(AX141150Plugin::is_bluetooth_address("/dev/rfcomm0"));

	AX141150Plugin plugin("COM5", AX141150Plugin::Mode::Bridge, "bad");
	EXPECT_EQ(AX141150Plugin::DEFAULT_PIN, plugin.get_pin());
}

#ifdef __linux__
/// Acts as the converter on the master side of a pseudo terminal: acknowledges every configuration command
/// and records what the driver sent.
class FakeConverter
{
public:
	FakeConverter()
	{
		master = posix_openpt(O_RDWR | O_NOCTTY);
		if ((-1 != master) && (0 == grantpt(master)) && (0 == unlockpt(master)))
		{
			slavePath = ptsname(master);
			worker = std::thread([this]() { run(); });
		}
	}

	~FakeConverter()
	{
		running = false;
		if (worker.joinable())
		{
			worker.join();
		}
		close(master);
	}

	void send(const std::vector<std::uint8_t> &message) const
	{
		EXPECT_EQ(static_cast<ssize_t>(message.size()), write(master, message.data(), message.size()));
	}

	std::vector<std::vector<std::uint8_t>> received_messages()
	{
		std::lock_guard<std::mutex> lock(receivedMutex);
		return received;
	}

	int master = -1;
	std::string slavePath;

private:
	void run()
	{
		std::vector<std::uint8_t> buffer;
		while (running)
		{
			struct pollfd descriptor = { master, POLLIN, 0 };
			std::uint8_t chunk[256];
			if ((poll(&descriptor, 1, 10) > 0) && (0 != (descriptor.revents & POLLIN)))
			{
				const ssize_t count = read(master, chunk, sizeof(chunk));
				if (count > 0)
				{
					buffer.insert(buffer.end(), chunk, chunk + count);
				}
			}

			while ((buffer.size() >= 2) && (buffer.size() >= (buffer[1] + 4u)))
			{
				const std::vector<std::uint8_t> message(buffer.begin(), buffer.begin() + buffer[1] + 4);
				buffer.erase(buffer.begin(), buffer.begin() + buffer[1] + 4);
				{
					std::lock_guard<std::mutex> lock(receivedMutex);
					received.push_back(message);
				}
				if (static_cast<std::uint8_t>(AX141150Plugin::MessageType::ChangeConfiguration) == message[0])
				{
					send(device_message(AX141150Plugin::MessageType::ChangeConfigurationResponse, { message[2] }));
				}
			}
		}
	}

	std::thread worker;
	std::atomic_bool running = { true };
	std::mutex receivedMutex;
	std::vector<std::vector<std::uint8_t>> received;
};

TEST(AX141150_PLUGIN_TESTS, OpensConfiguresAndExchangesFrames)
{
	FakeConverter converter;
	ASSERT_FALSE(converter.slavePath.empty());

	AX141150Plugin plugin(converter.slavePath, AX141150Plugin::Mode::Bridge, "123456");
	plugin.open();
	ASSERT_TRUE(plugin.get_is_valid());
	EXPECT_EQ((std::vector<std::uint8_t>{ 0xC1 }), plugin.get_last_configuration_response());

	const std::vector<std::vector<std::uint8_t>> configuration = converter.received_messages();
	ASSERT_EQ(3u, configuration.size());
	EXPECT_EQ(AX141150Plugin::encode_message(0x67, { 0xC0, '1', '2', '3', '4', '5', '6' }), configuration[0]);
	EXPECT_EQ(AX141150Plugin::encode_message(0x67, { 0x03 }), configuration[1]);
	EXPECT_EQ(AX141150Plugin::encode_message(0x67, { 0xC1 }), configuration[2]);

	CANMessageFrame frame = {};
	frame.identifier = 0x18EFFF80;
	frame.isExtendedFrame = true;
	frame.dataLength = 1;
	frame.data[0] = 0x42;
	EXPECT_TRUE(plugin.write_frame(frame));

	converter.send(device_message(AX141150Plugin::MessageType::CANDataExtendedFromDevice, { 0x81, 0xFF, 0xEF, 0x18, 1, 0x24 }));
	CANMessageFrame received = {};
	bool gotFrame = false;
	for (int attempt = 0; (!gotFrame) && (attempt < 10); attempt++)
	{
		gotFrame = plugin.read_frame(received);
	}
	ASSERT_TRUE(gotFrame);
	EXPECT_EQ(0x18EFFF81u, received.identifier);
	EXPECT_EQ(0x24, received.data[0]);

	for (int attempt = 0; (converter.received_messages().size() < 4u) && (attempt < 100); attempt++)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	ASSERT_EQ(4u, converter.received_messages().size());
	EXPECT_EQ(AX141150Plugin::encode_can_frame(frame), converter.received_messages()[3]);

	plugin.close();
	EXPECT_FALSE(plugin.get_is_valid());
	EXPECT_FALSE(plugin.write_frame(frame));
}
#endif
