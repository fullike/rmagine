#include "rmagine/map/optix/OptixTexture.hpp"
#include "rmagine/util/optix/OptixDebug.hpp"

namespace rmagine
{

OptixTexture::OptixTexture(OptixContextPtr context)
:Base(context)
{
    // std::cout << "[OptixMesh::OptixMesh()] constructed." << std::endl;
}

OptixTexture::~OptixTexture()
{

}

void OptixTexture::init(int width, int height, uint32_t* data)
{
    cudaResourceDesc res_desc = {};
    cudaChannelFormatDesc channel_desc;
    int32_t numComponents = 4;
    int32_t pitch  = width*numComponents*sizeof(uint8_t);
    channel_desc = cudaCreateChannelDesc<uchar4>();

    RM_CUDA_CHECK(cudaMallocArray(&pixels,
                            &channel_desc,
                            width,height));

    RM_CUDA_CHECK(cudaMemcpy2DToArray(pixels,
                                /* offset */0,0,
                                data,
                                pitch,pitch,height,
                                cudaMemcpyHostToDevice));

    res_desc.resType          = cudaResourceTypeArray;
    res_desc.res.array.array  = pixels;

    cudaTextureDesc tex_desc     = {};
    tex_desc.addressMode[0]      = cudaAddressModeWrap;
    tex_desc.addressMode[1]      = cudaAddressModeWrap;
    tex_desc.filterMode          = cudaFilterModeLinear;
    tex_desc.readMode            = cudaReadModeNormalizedFloat;
    tex_desc.normalizedCoords    = 1;
    tex_desc.maxAnisotropy       = 1;
    tex_desc.maxMipmapLevelClamp = 99;
    tex_desc.minMipmapLevelClamp = 0;
    tex_desc.mipmapFilterMode    = cudaFilterModePoint;
    tex_desc.borderColor[0]      = 1.0f;
    tex_desc.sRGB                = 0;

    // Create texture object
    RM_CUDA_CHECK(cudaCreateTextureObject(&object, &res_desc, &tex_desc, nullptr));
}

}