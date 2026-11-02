/** Tests for replacing the depth processor while packets arrive, and for the depth frame stride.
 *
 * Needs no Kinect: depth packets are synthesized and fed to the pipeline's stream parser, the way the
 * USB transfer callback does. DumpPacketPipeline forwards every packet as a raw frame, which makes
 * the number of processed packets observable through a FrameListener.
 */

#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/data_callback.h>
#include <libfreenect2/depth_packet_processor.h>
#include <libfreenect2/depth_packet_stream_parser.h>
#include <libfreenect2/frame_listener.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

using namespace libfreenect2;

namespace
{

int failures = 0;

#define CHECK(cond)                                                              \
	do                                                                           \
	{                                                                            \
		if (!(cond))                                                             \
		{                                                                        \
			std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
			++failures;                                                          \
		}                                                                        \
	} while (false)

class CountingListener : public FrameListener
{
public:
	std::atomic<int> depth_frames{0};

	bool onNewFrame(Frame::Type type, Frame*) override
	{
		if (type == Frame::Depth)
			++depth_frames;
		return false; // the library keeps ownership and frees the frame
	}
};

/** One depth packet is ten subpackets; the packet completes when the next sequence starts. */
void feed_sequence(DataCallback& parser, uint32_t sequence)
{
	constexpr size_t single_image = 512 * 424 * 11 / 8;
	std::vector<unsigned char> chunk(single_image + sizeof(DepthSubPacketFooter));
	for (uint32_t sub = 0; sub < 10; ++sub)
	{
		auto* footer = reinterpret_cast<DepthSubPacketFooter*>(chunk.data() + single_image);
		std::memset(footer, 0, sizeof(*footer));
		footer->timestamp = sequence * 8;
		footer->sequence = sequence;
		footer->subsequence = sub;
		footer->length = single_image;
		parser.onDataReceived(chunk.data(), chunk.size());
	}
}

/** The dump processor copies a packet in well under a millisecond; the pause lets its worker go idle. */
void feed(DataCallback& parser, uint32_t first, uint32_t count)
{
	for (uint32_t i = 0; i < count; ++i)
	{
		feed_sequence(parser, first + i);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void test_replace_hands_over_state()
{
	DumpPacketPipeline target;
	DumpPacketPipeline donor;
	DepthPacketProcessor* incoming = donor.getDepthPacketProcessor();
	DepthPacketProcessor* outgoing = target.getDepthPacketProcessor();
	CHECK(incoming != nullptr && outgoing != nullptr && incoming != outgoing);

	const unsigned char p0[] = {1, 2, 3, 4};
	DepthPacketProcessor* prepared = nullptr;
	bool ok = target.replaceDepthProcessor(donor, [&](DepthPacketProcessor& proc)
	{
		prepared = &proc;
		proc.loadP0TablesFromCommandResponse(const_cast<unsigned char*>(p0), sizeof(p0));
	});

	CHECK(ok);
	CHECK(prepared == incoming);
	CHECK(target.getDepthPacketProcessor() == incoming);
	CHECK(donor.getDepthPacketProcessor() == nullptr);

	size_t length = 0;
	const unsigned char* tables = target.getDepthP0Tables(&length);
	CHECK(length > 0 && tables != nullptr && std::memcmp(tables, p0, sizeof(p0)) == 0);

	// The donor is spent now, and a pipeline is no donor for itself: both change nothing
	CHECK(!target.replaceDepthProcessor(donor, nullptr));
	CHECK(target.getDepthPacketProcessor() == incoming);
	CHECK(!target.replaceDepthProcessor(target, nullptr));
	CHECK(target.getDepthPacketProcessor() == incoming);
}

void test_stream_continues_after_replace()
{
	CountingListener listener;
	DumpPacketPipeline pipeline;
	pipeline.getDepthPacketProcessor()->setFrameListener(&listener);
	DataCallback& parser = *pipeline.getIrPacketParser();

	feed(parser, 1, 6); // five packets complete
	CHECK(listener.depth_frames == 5);

	DumpPacketPipeline donor;
	CHECK(pipeline.replaceDepthProcessor(donor, [&](DepthPacketProcessor& proc) { proc.setFrameListener(&listener); }));

	feed(parser, 7, 6); // the first of these completes sequence 6, still from before the swap
	CHECK(listener.depth_frames == 11);
}

void test_frame_stride()
{
	CountingListener listener;
	DumpPacketPipeline pipeline;
	pipeline.getDepthPacketProcessor()->setFrameListener(&listener);
	DataCallback& parser = *pipeline.getIrPacketParser();

	pipeline.setDepthFrameStride(3);
	feed(parser, 1, 31); // thirty packets complete, every third is processed
	CHECK(listener.depth_frames == 10);

	pipeline.setDepthFrameStride(0);
	feed(parser, 32, 10);
	CHECK(listener.depth_frames == 10);

	pipeline.setDepthFrameStride(1);
	feed(parser, 42, 6); // completes sequences 41 to 46
	CHECK(listener.depth_frames == 16);
}

/** Swapping in a loop against a thread that keeps feeding packets: must neither crash nor deadlock. */
void test_replace_under_load()
{
	CountingListener listener;
	DumpPacketPipeline pipeline;
	pipeline.getDepthPacketProcessor()->setFrameListener(&listener);
	DataCallback& parser = *pipeline.getIrPacketParser();

	std::atomic<bool> stop{false};
	std::thread producer([&]
	{
		for (uint32_t sequence = 1; !stop; ++sequence)
		{
			feed_sequence(parser, sequence);
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	});

	constexpr int swaps = 200;
	int succeeded = 0;
	for (int i = 0; i < swaps; ++i)
	{
		DumpPacketPipeline donor;
		if (pipeline.replaceDepthProcessor(donor, [&](DepthPacketProcessor& proc) { proc.setFrameListener(&listener); }))
			++succeeded;
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}

	stop = true;
	producer.join();
	CHECK(succeeded == swaps);
	CHECK(listener.depth_frames > 0);
}

} // namespace

int main()
{
	const struct
	{
		const char* name;
		void (*run)();
	} tests[] = {
		{"replace hands over state", test_replace_hands_over_state},
		{"stream continues after replace", test_stream_continues_after_replace},
		{"frame stride", test_frame_stride},
		{"replace under load", test_replace_under_load},
	};

	for (const auto& test : tests)
	{
		std::fprintf(stderr, "[ RUN ] %s\n", test.name);
		test.run();
	}

	if (failures != 0)
	{
		std::fprintf(stderr, "%d check(s) failed\n", failures);
		return EXIT_FAILURE;
	}
	std::puts("pipeline_swap_test: ok");
	return EXIT_SUCCESS;
}
