#ifndef RMAGINE_MAP_OPTIX_MATERIAL_HPP
#define RMAGINE_MAP_OPTIX_MATERIAL_HPP
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

class OptixMaterial : public OptixEntity
{
public:
    using Base = OptixEntity;

    OptixMaterial(OptixContextPtr context = optix_default_context());

    virtual ~OptixMaterial();

};


}
#endif // RMAGINE_MAP_OPTIX_MATERIAL_HPP