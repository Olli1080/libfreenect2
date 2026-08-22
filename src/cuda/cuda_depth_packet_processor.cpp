#include "internal/cuda/cuda_depth_packet_processor.h"

#include <internal/libfreenect2/protocol/response.h>
#include "internal/libfreenect2/logging.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define CHECK_CUDA(expr) do { cudaError_t err = (expr); if (err != cudaSuccess) { LOG_ERROR("{}: {}", #expr, cudaGetErrorString(err)); return false; } } while(0)
#define CALL_CUDA(expr) do { cudaError_t err = (expr); if (err != cudaSuccess) { LOG_ERROR("{}: {}", #expr, cudaGetErrorString(err)); } } while(0)

namespace libfreenect2
{
    CudaFrame::CudaFrame(Buffer* buffer) :
        Frame(512, 424, 4, (unsigned char*)-1)
    {
        data = buffer->data;
        rawdata = reinterpret_cast<unsigned char*>(buffer);
    }

    CudaFrame::~CudaFrame()
    {
        Buffer* buffer = reinterpret_cast<Buffer*>(rawdata);
        buffer->allocator->free(buffer);
        rawdata = nullptr;
    }

    bool CudaAllocator::allocate_cuda(Buffer* b, size_t size)
    {
        unsigned int flags = cudaHostAllocPortable;
        if (input)
            flags |= cudaHostAllocWriteCombined;
        CHECK_CUDA(cudaHostAlloc(&b->data, size, flags));
        b->length = 0;
        b->capacity = size;
        return true;
    }

    Buffer* CudaAllocator::allocate(size_t size)
    {
        auto b = std::make_unique<Buffer>();
        if (!allocate_cuda(b.get(), size))
            b->data = nullptr;
        return b.release();
    }

    void CudaAllocator::free(Buffer* b)
    {
        if (b == nullptr)
            return;
        std::unique_ptr<Buffer> buffer(b);
        if (buffer->data)
            CALL_CUDA(cudaFreeHost(buffer->data));
    }

    CudaDepthPacketProcessorImpl::CudaDepthPacketProcessorImpl(const int deviceId) :
        block_size(128),
        grid_size(IMAGE_SIZE / block_size),
        config(),
        params(),
        ir_frame(nullptr),
        depth_frame(nullptr),
        input_allocator(nullptr),
        ir_allocator(nullptr),
        depth_allocator(nullptr)
    {
        good = initDevice(deviceId);
        if (!good)
            return;

        input_allocator = std::make_unique<PoolAllocator>(new CudaAllocator(true));
        ir_allocator = std::make_unique<PoolAllocator>(new CudaAllocator(false));
        depth_allocator = std::make_unique<PoolAllocator>(new CudaAllocator(false));

        newIrFrame();
        newDepthFrame();
    }

    CudaDepthPacketProcessorImpl::~CudaDepthPacketProcessorImpl()
    {
        // Reset in this order: CudaFrame's destructor calls the allocator's free()
        ir_frame.reset();
        depth_frame.reset();
        input_allocator.reset();
        ir_allocator.reset();
        depth_allocator.reset();
        if (good)
            freeDeviceMemory();
    }

    bool CudaDepthPacketProcessorImpl::allocateDeviceMemory()
    {
        //Read only
        d_p0table_size = IMAGE_SIZE * sizeof(float4);
        d_xtable_size = IMAGE_SIZE * sizeof(float);
        d_ztable_size = IMAGE_SIZE * sizeof(float);
        d_lut_size = LUT_SIZE * sizeof(short);

        CHECK_CUDA(cudaMalloc(&d_p0table, d_p0table_size));
        CHECK_CUDA(cudaMalloc(&d_xtable, d_xtable_size));
        CHECK_CUDA(cudaMalloc(&d_ztable, d_ztable_size));
        CHECK_CUDA(cudaMalloc(&d_lut, d_lut_size));

        d_packet_size = (IMAGE_SIZE * 11 / 8) * 10;

        CHECK_CUDA(cudaMalloc(&d_packet, d_packet_size));

        //Read-Write
        size_t d_a_size = IMAGE_SIZE * sizeof(float4);
        size_t d_b_size = IMAGE_SIZE * sizeof(float4);
        size_t d_n_size = IMAGE_SIZE * sizeof(float4);
        size_t d_ir_size = IMAGE_SIZE * sizeof(float);
        size_t d_a_filtered_size = IMAGE_SIZE * sizeof(float4);
        size_t d_b_filtered_size = IMAGE_SIZE * sizeof(float4);
        size_t d_edge_test_size = IMAGE_SIZE * sizeof(char);
        size_t d_depth_size = IMAGE_SIZE * sizeof(float);
        size_t d_ir_sum_size = IMAGE_SIZE * sizeof(float);
        size_t d_filtered_size = IMAGE_SIZE * sizeof(float);

        CHECK_CUDA(cudaMalloc(&d_a, d_a_size));
        CHECK_CUDA(cudaMalloc(&d_b, d_b_size));
        CHECK_CUDA(cudaMalloc(&d_n, d_n_size));
        CHECK_CUDA(cudaMalloc(&d_ir, d_ir_size));
        CHECK_CUDA(cudaMalloc(&d_a_filtered, d_a_filtered_size));
        CHECK_CUDA(cudaMalloc(&d_b_filtered, d_b_filtered_size));
        CHECK_CUDA(cudaMalloc(&d_edge_test, d_edge_test_size));
        CHECK_CUDA(cudaMalloc(&d_depth, d_depth_size));
        CHECK_CUDA(cudaMalloc(&d_ir_sum, d_ir_sum_size));
        CHECK_CUDA(cudaMalloc(&d_filtered, d_filtered_size));

        cudaDeviceSynchronize();

        CHECK_CUDA(cudaGetLastError());
        return true;
    }

    void CudaDepthPacketProcessorImpl::freeDeviceMemory()
    {
        CALL_CUDA(cudaFree(d_p0table));
        CALL_CUDA(cudaFree(d_xtable));
        CALL_CUDA(cudaFree(d_ztable));
        CALL_CUDA(cudaFree(d_lut));

        CALL_CUDA(cudaFree(d_packet));

        CALL_CUDA(cudaFree(d_a));
        CALL_CUDA(cudaFree(d_b));
        CALL_CUDA(cudaFree(d_n));
        CALL_CUDA(cudaFree(d_ir));
        CALL_CUDA(cudaFree(d_a_filtered));
        CALL_CUDA(cudaFree(d_b_filtered));
        CALL_CUDA(cudaFree(d_edge_test));
        CALL_CUDA(cudaFree(d_depth));
        CALL_CUDA(cudaFree(d_ir_sum));
        CALL_CUDA(cudaFree(d_filtered));
    }

    bool CudaDepthPacketProcessorImpl::initDevice(const int deviceId)
    {
        int deviceCount = 0;

        CHECK_CUDA(cudaGetDeviceCount(&deviceCount));

        int devId = -1;
        for (int i = 0; i < deviceCount; i++) {
            if (deviceId != -1 && i != deviceId)
                continue;

            cudaDeviceProp prop;
            CHECK_CUDA(cudaGetDeviceProperties(&prop, i));

            int clockRate_khz = 0;
            CHECK_CUDA(cudaDeviceGetAttribute(&clockRate_khz, cudaDevAttrClockRate, i));

            int computeMode = 0;
            CHECK_CUDA(cudaDeviceGetAttribute(&computeMode, cudaDevAttrComputeMode, i));

            LOG_INFO("device {}: {} @ {}MHz Memory {}MB", i, prop.name, (clockRate_khz / 1000), (prop.totalGlobalMem >> 20));

            if (computeMode == cudaComputeModeProhibited) {
                LOG_INFO(" Compute Mode Prohibited");
                continue;
            }

            if (prop.major < 1) {
                LOG_INFO(" does not support CUDA");
                continue;
            }

            devId = i;
            break;
        }

        if (devId == -1) {
            LOG_ERROR("No CUDA device found");
            return false;
        }

        CHECK_CUDA(cudaSetDevice(devId));
        LOG_INFO("selected device {}", devId);

        if (!setParameters(params))
            return false;

        if (!setConfiguration(config))
            return false;

        if (!allocateDeviceMemory())
            return false;

        return true;
    }    

    bool CudaDepthPacketProcessorImpl::last_call_success() const
    {
        CHECK_CUDA(cudaGetLastError());
        return true;
    }

    void CudaDepthPacketProcessorImpl::newIrFrame()
    {
        ir_frame = std::make_unique<CudaFrame>(ir_allocator->allocate(IMAGE_SIZE * sizeof(float)));
        ir_frame->format = Frame::Float;
    }

    void CudaDepthPacketProcessorImpl::newDepthFrame()
    {
        depth_frame = std::make_unique<CudaFrame>(depth_allocator->allocate(IMAGE_SIZE * sizeof(float)));
        depth_frame->format = Frame::Float;
    }

    void CudaDepthPacketProcessorImpl::fill_trig_table(const protocol::P0TablesResponse* p0table)
    {
        for (int r = 0; r < 424; ++r) {
            float4* it = &h_p0table[r * 512];
            const uint16_t* it0 = &p0table->p0table0[r * 512];
            const uint16_t* it1 = &p0table->p0table1[r * 512];
            const uint16_t* it2 = &p0table->p0table2[r * 512];
            for (int c = 0; c < 512; ++c, ++it, ++it0, ++it1, ++it2) {
                it->x = -((float)*it0) * 0.000031 * M_PI;
                it->y = -((float)*it1) * 0.000031 * M_PI;
                it->z = -((float)*it2) * 0.000031 * M_PI;
                it->w = 0.0f;
            }
        }
    }




    CudaDepthPacketProcessor::CudaDepthPacketProcessor(const int deviceId) :
        impl_(std::make_unique<CudaDepthPacketProcessorImpl>(deviceId))
    {
    }

    CudaDepthPacketProcessor::~CudaDepthPacketProcessor()
    {
    }

    void CudaDepthPacketProcessor::setConfiguration(const DepthPacketProcessor::Config& config)
    {
        DepthPacketProcessor::setConfiguration(config);

        impl_->good = impl_->setConfiguration(config);
    }

    void CudaDepthPacketProcessor::loadP0TablesFromCommandResponse(unsigned char* buffer, size_t buffer_length)
    {
        impl_->fill_trig_table((protocol::P0TablesResponse*)buffer);
        cudaMemcpy(impl_->d_p0table, impl_->h_p0table, impl_->d_p0table_size, cudaMemcpyHostToDevice);
    }

    void CudaDepthPacketProcessor::loadXZTables(const float* xtable, const float* ztable)
    {
        cudaMemcpy(impl_->d_xtable, xtable, impl_->d_xtable_size, cudaMemcpyHostToDevice);
        cudaMemcpy(impl_->d_ztable, ztable, impl_->d_ztable_size, cudaMemcpyHostToDevice);
    }

    void CudaDepthPacketProcessor::loadLookupTable(const short* lut)
    {
        cudaMemcpy(impl_->d_lut, lut, impl_->d_lut_size, cudaMemcpyHostToDevice);
    }

    bool CudaDepthPacketProcessor::good() const
    {
        return impl_->good;
    }

    void CudaDepthPacketProcessor::process(const DepthPacket& packet)
    {
        if (listener_ == nullptr)
            return;

        impl_->startTiming();

        impl_->ir_frame->timestamp = packet.timestamp;
        impl_->depth_frame->timestamp = packet.timestamp;
        impl_->ir_frame->sequence = packet.sequence;
        impl_->depth_frame->sequence = packet.sequence;

        impl_->good = impl_->run(packet);

        std::string timing = impl_->stopTiming(LOG_SOURCE);
        if (!timing.empty())
            LOG_INFO("{}", timing);

        if (!impl_->good) {
            impl_->ir_frame->status = 1;
            impl_->depth_frame->status = 1;
        }

        if (listener_->onNewFrame(Frame::Ir, impl_->ir_frame.get()))
        {
            impl_->ir_frame.release();
            impl_->newIrFrame();
        }
        if (listener_->onNewFrame(Frame::Depth, impl_->depth_frame.get()))
        {
            impl_->depth_frame.release();
            impl_->newDepthFrame();
        }
    }

    Allocator* CudaDepthPacketProcessor::getAllocator()
    {
        return impl_->input_allocator.get();
    }
} // namespace libfreenect2
