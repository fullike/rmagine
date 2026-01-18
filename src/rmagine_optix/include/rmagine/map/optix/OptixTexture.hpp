#ifndef RMAGINE_MAP_OPTIX_TEXTURE_HPP
#define RMAGINE_MAP_OPTIX_TEXTURE_HPP
#include <cuda_runtime.h>

#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/math/types.h>
#include <rmagine/types/mesh_types.h>

#include <rmagine/util/cuda/CudaContext.hpp>
#include <rmagine/util/optix/OptixContext.hpp>

#include <memory>

#include <assimp/mesh.h>

#include "OptixNode.hpp"

#include "optix_definitions.h"

#include "optix_sbt.h"

namespace rmagine
{

class OptixTexture : public OptixEntity
{
public:
    using Base = OptixEntity;

    OptixTexture(OptixContextPtr context = optix_default_context());

    virtual ~OptixTexture();

    void init(int width, int height, uint32_t* data);

    cudaArray_t pixels;
    cudaTextureObject_t object;
};


}
#endif // RMAGINE_MAP_OPTIX_TEXTURE_HPP