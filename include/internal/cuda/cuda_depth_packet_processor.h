#pragma once

#include <internal/libfreenect2/depth_packet_processor.h>
#include <internal/libfreenect2/perf_logging.h>
#include <freenect2_cuda/export.h>

#include <cuda_runtime.h>

namespace libfreenect2
{
	namespace protocol
	{
		struct P0TablesResponse;
	}

	class CudaFrame : public Frame
    {
    public:
        CudaFrame(Buffer* buffer);

        ~CudaFrame() override;
    };

    class CudaAllocator : public Allocator
    {
    private:
        const bool input;

        bool allocate_cuda(Buffer* b, size_t size);

    public:
        CudaAllocator(bool input) : input(input) {}

        Buffer* allocate(size_t size) override;

        void free(Buffer* b) override;
    };

    class CudaDepthPacketProcessorImpl : public WithPerfLogging
    {
    public:
        static const size_t IMAGE_SIZE = 512 * 424;
        static const size_t LUT_SIZE = 2048;

        size_t d_lut_size;
        size_t d_xtable_size;
        size_t d_ztable_size;
        size_t d_p0table_size;

        short* d_lut;
        float* d_xtable;
        float* d_ztable;
        float4* d_p0table;
        float4 h_p0table[IMAGE_SIZE];

        size_t d_packet_size;
        unsigned short* d_packet;

        float4* d_a;
        float4* d_b;
        float4* d_n;
        float* d_ir;
        float4* d_a_filtered;
        float4* d_b_filtered;
        unsigned char* d_edge_test;
        float* d_depth;
        float* d_ir_sum;
        float* d_filtered;

        size_t block_size;
        size_t grid_size;

        DepthPacketProcessor::Config config;
        DepthPacketProcessor::Parameters params;

        std::unique_ptr<Frame> ir_frame;
        std::unique_ptr<Frame> depth_frame;

        std::unique_ptr<Allocator> input_allocator;
        std::unique_ptr<Allocator> ir_allocator;
        std::unique_ptr<Allocator> depth_allocator;

        bool good;

        CudaDepthPacketProcessorImpl(const int deviceId);

        ~CudaDepthPacketProcessorImpl();// override;

        bool setParameters(const DepthPacketProcessor::Parameters& params);
        bool last_call_success() const;

        bool allocateDeviceMemory();

        void freeDeviceMemory();

        bool initDevice(const int deviceId);

        bool setConfiguration(const DepthPacketProcessor::Config& cfg);

        bool run(const DepthPacket& packet);

        void newIrFrame();

        void newDepthFrame();

        void fill_trig_table(const protocol::P0TablesResponse* p0table);
    };

    class FREENECT2_CUDA_EXPORT CudaDepthPacketProcessor : public DepthPacketProcessor
    {
    public:
        CudaDepthPacketProcessor(const int deviceId = -1);
        ~CudaDepthPacketProcessor() noexcept override;
        void setConfiguration(const libfreenect2::DepthPacketProcessor::Config& config) override;

        void loadP0TablesFromCommandResponse(unsigned char* buffer, size_t buffer_length) override;

        void loadXZTables(const float* xtable, const float* ztable) override;
        void loadLookupTable(const short* lut) override;

        bool good() const override;
        std::string_view name() override { return "CUDA"; }

        void process(const DepthPacket& packet) override;
    protected:
        Allocator* getAllocator() override;
    private:
        std::unique_ptr<CudaDepthPacketProcessorImpl> impl_;
    };
} // namespace libfreenect2
