#include "rmagine/map/optix/OptixMaterial.hpp"
#include "rmagine/util/optix/OptixDebug.hpp"

namespace rmagine
{

OptixMaterial::OptixMaterial(OptixContextPtr context)
:Base(context)
{
    // std::cout << "[OptixMesh::OptixMesh()] constructed." << std::endl;
}

OptixMaterial::~OptixMaterial()
{

}

}