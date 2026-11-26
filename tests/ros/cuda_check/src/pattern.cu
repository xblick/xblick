// The test pattern of cuda_check.cpp, checked on the GPU: byte i = (i * 13 + seed) & 0xFF.
#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>

__global__ void pattern_check_kernel(const uint8_t* p, size_t n, uint32_t seed, unsigned long long* bad)
{
    unsigned long long local = 0;
    for (size_t i = blockIdx.x * blockDim.x + threadIdx.x; i < n; i += size_t(gridDim.x) * blockDim.x)
    {
        local += p[i] != static_cast<uint8_t>((i * 13u + seed) & 0xFFu);
    }
    atomicAdd(bad, local);
}

cudaError_t pattern_check(const void* p, size_t n, uint32_t seed, cudaStream_t s, unsigned long long* bad)
{
    unsigned long long* d = nullptr;
    cudaError_t e = cudaMallocAsync(reinterpret_cast<void**>(&d), sizeof(unsigned long long), s);
    if (e != cudaSuccess)
    {
        return e;
    }
    cudaMemsetAsync(d, 0, sizeof(unsigned long long), s);
    pattern_check_kernel<<<256, 256, 0, s>>>(static_cast<const uint8_t*>(p), n, seed, d);
    cudaMemcpyAsync(bad, d, sizeof(unsigned long long), cudaMemcpyDeviceToHost, s);
    cudaFreeAsync(d, s);
    e = cudaStreamSynchronize(s);
    return e != cudaSuccess ? e : cudaGetLastError();
}
