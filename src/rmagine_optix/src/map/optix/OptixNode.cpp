#include "rmagine/map/optix/OptixNode.hpp"
#include <rmagine/math/linalg.h>
#include <rmagine/util/optix/OptixDebug.hpp>
#include "rmagine/map/optix/OptixAccelerationStructure.hpp"
#include "rmagine/map/optix/OptixScene.hpp"
#include "rmagine/map/optix/OptixMesh.hpp"
#include <optix_stubs.h>
#include <optix_types.h>
namespace rmagine
{

OptixNode::OptixNode(OptixContextPtr context)
:OptixEntity(context)
,OptixTransformable()
{
    // std::cout << "[OptixGeometry::OptixGeometry()] constructed." << std::endl;
}

OptixNode::~OptixNode()
{
    // std::cout << "[OptixGeometry::~OptixGeometry()] destroyed." << std::endl;
}

void OptixNode::buildIAS()
{
    if (m_mesh)
    {
        m_as = m_mesh->as();
        return;
    }
    const size_t n_instances = m_children.size();
    // fill m_hitgroup_data
    Memory<OptixInstance, RAM> inst_h(n_instances);
    for(int idx = 0; idx < n_instances; idx++)
    {
        OptixNodePtr child = m_children[idx];
        child->buildIAS();
        OptixInstance inst = {};
        Matrix4x4 M = child->matrix();
        inst.transform[ 0] = M(0,0); // Rxx
        inst.transform[ 1] = M(0,1); // Rxy
        inst.transform[ 2] = M(0,2); // Rxz
        inst.transform[ 3] = M(0,3); // tx
        inst.transform[ 4] = M(1,0); // Ryx
        inst.transform[ 5] = M(1,1); // Ryy
        inst.transform[ 6] = M(1,2); // Ryz
        inst.transform[ 7] = M(1,3); // ty
        inst.transform[ 8] = M(2,0); // Rzx
        inst.transform[ 9] = M(2,1); // Rzy
        inst.transform[10] = M(2,2); // Rzz
        inst.transform[11] = M(2,3); // tz
        inst.traversableHandle = child->as()->handle;
        inst.instanceId = idx;
        inst.visibilityMask = 1;
        inst_h[idx] = inst;
    }

    // std::cout << "- COPY INSTANCE DATA" << std::endl;

    // COPY INSTANCES DATA
    CUdeviceptr m_inst_buffer;
    RM_CUDA_CHECK( cudaMalloc(
        reinterpret_cast<void**>( &m_inst_buffer ),
        inst_h.size() * sizeof(OptixInstance) ) );

    RM_CUDA_CHECK( cudaMemcpyAsync(
                reinterpret_cast<void*>( m_inst_buffer ),
                inst_h.raw(),
                inst_h.size() * sizeof(OptixInstance),
                cudaMemcpyHostToDevice,
                m_stream->handle()
                ) );
    // std::cout << "- MAKE BUILD INPUT" << std::endl;
    // BEGIN WITH BUILD INPUT
    OptixBuildInput instance_input = {};
    instance_input.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES;
    instance_input.instanceArray.numInstances = inst_h.size();
    instance_input.instanceArray.instances = m_inst_buffer;

    OptixAccelBuildOptions ias_accel_options = {};
    unsigned int build_flags = OPTIX_BUILD_FLAG_NONE;
    { // BUILD FLAGS
        build_flags |= OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
        build_flags |= OPTIX_BUILD_FLAG_ALLOW_UPDATE;
        #if OPTIX_VERSION >= 70300
        build_flags |= OPTIX_BUILD_FLAG_ALLOW_RANDOM_INSTANCE_ACCESS;
        #endif
    }

    ias_accel_options.buildFlags = build_flags;
    ias_accel_options.motionOptions.numKeys = 1;
    ias_accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

    OptixAccelBufferSizes ias_buffer_sizes;
    RM_OPTIX_CHECK( optixAccelComputeMemoryUsage(
        m_ctx->ref(),
        &ias_accel_options,
        &instance_input,
        1,
        &ias_buffer_sizes ) );


    CUdeviceptr d_temp_buffer_ias;
    RM_CUDA_CHECK( cudaMalloc(
        reinterpret_cast<void**>( &d_temp_buffer_ias ),
        ias_buffer_sizes.tempSizeInBytes) );


    if(!m_as)
    {
        // make new
        m_as = std::make_shared<OptixAccelerationStructure>();
        RM_CUDA_CHECK( cudaMalloc(
                reinterpret_cast<void**>( &m_as->buffer ),
                ias_buffer_sizes.outputSizeInBytes
                ) );
    } else {
        if(m_as->buffer_size != ias_buffer_sizes.outputSizeInBytes)
        {
            // realloc
            RM_CUDA_CHECK( cudaFree( reinterpret_cast<void*>( m_as->buffer ) ) );
            RM_CUDA_CHECK( cudaMalloc(
                    reinterpret_cast<void**>( &m_as->buffer ),
                    ias_buffer_sizes.outputSizeInBytes
                    ) );
        }
    }

    m_as->buffer_size = ias_buffer_sizes.outputSizeInBytes;
    m_as->n_elements = n_instances;


    RM_OPTIX_CHECK(optixAccelBuild(
        m_ctx->ref(),
        m_stream->handle(),
        &ias_accel_options,
        &instance_input,
        1, // num build inputs
        d_temp_buffer_ias,
        ias_buffer_sizes.tempSizeInBytes,
        m_as->buffer,
        ias_buffer_sizes.outputSizeInBytes,
        &m_as->handle,
        nullptr,
        0
    ));

    RM_CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_temp_buffer_ias ) ) );
}

} // namespace rmagine