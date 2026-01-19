#include "rmagine/map/optix/OptixMesh.hpp"

#include "rmagine/map/optix/OptixAccelerationStructure.hpp"

#include "rmagine/math/assimp_conversions.h"
#include "rmagine/util/optix/OptixDebug.hpp"
#include "rmagine/types/MemoryCuda.hpp"
#include "rmagine/util/GenericAlign.hpp"
#include "rmagine/map/mesh_preprocessing.cuh"
#include "rmagine/math/memory_math.cuh"

#include "rmagine/util/cuda/CudaStream.hpp"

#include <optix.h>
#include <optix_stubs.h>

#include <cuda.h>
#include <cuda_runtime.h>

#include <rmagine/util/prints.h>

namespace rmagine
{

OptixMesh::OptixMesh(OptixContextPtr context)
:Base(context)
{
    // std::cout << "[OptixMesh::OptixMesh()] constructed." << std::endl;
    local_scale = {1.,1.,1.};
}

OptixMesh::~OptixMesh()
{
}

void OptixMesh::apply()
{
}

void OptixMesh::commit()
{
    m_vertices_ref = reinterpret_cast<CUdeviceptr>(vertices.raw());

    sbt_data.vertex_normals = vertex_normals.raw();
    sbt_data.vertex_uvs = vertex_uvs.raw();
}

unsigned int OptixMesh::depth() const
{
    return 0;
}

void OptixMesh::buildGAS()
{
    OptixBuildInput triangle_input = {};
    triangle_input.type                        = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
    // VERTICES
    triangle_input.triangleArray.vertexFormat  = OPTIX_VERTEX_FORMAT_FLOAT3;
    triangle_input.triangleArray.vertexStrideInBytes = sizeof(Point);
    triangle_input.triangleArray.numVertices   = vertices.size();
    triangle_input.triangleArray.vertexBuffers = getVertexBuffer();
    // FACES
    triangle_input.triangleArray.indexFormat  = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
    triangle_input.triangleArray.indexStrideInBytes  = sizeof(Face);
    triangle_input.triangleArray.numIndexTriplets    = faces.size();
    triangle_input.triangleArray.indexBuffer         = getFaceBuffer();
    // ADDITIONAL SETTINGS
    // move them to mesh object
    triangle_input.triangleArray.flags         = (const uint32_t [1]) {
        OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT
    };
    // TODO: this is bad. I define the sbt records inside the sensor programs.
    triangle_input.triangleArray.numSbtRecords = 1;

    // Acceleration Options
    // Use default options for simplicity.  In a real use case we would want to
    // enable compaction, etc
    OptixAccelBuildOptions accel_options = {};

    unsigned int build_flags = OPTIX_BUILD_FLAG_NONE;

    { // BUILD FLAGS
        build_flags |= OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
        build_flags |= OPTIX_BUILD_FLAG_ALLOW_UPDATE;
        build_flags |= OPTIX_BUILD_FLAG_ALLOW_RANDOM_VERTEX_ACCESS;
    }

    accel_options.buildFlags = build_flags;
    accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

    OptixAccelBufferSizes gas_buffer_sizes;
    RM_OPTIX_CHECK( optixAccelComputeMemoryUsage(
                m_ctx->ref(),
                &accel_options,
                &triangle_input,
                1, // Number of build inputs
                &gas_buffer_sizes
                ) );


    CUdeviceptr d_temp_buffer_gas;
    RM_CUDA_CHECK( cudaMalloc(
        reinterpret_cast<void**>( &d_temp_buffer_gas ),
        gas_buffer_sizes.tempSizeInBytes) );

    if(!m_as)
    {
        // make new
        m_as = std::make_shared<OptixAccelerationStructure>();
        RM_CUDA_CHECK( cudaMalloc(
                reinterpret_cast<void**>( &m_as->buffer ),
                gas_buffer_sizes.outputSizeInBytes
                ) );
    } else {
        if(m_as->buffer_size != gas_buffer_sizes.outputSizeInBytes)
        {
            // realloc
            RM_CUDA_CHECK( cudaFree( reinterpret_cast<void*>( m_as->buffer ) ) );
            RM_CUDA_CHECK( cudaMalloc(
                    reinterpret_cast<void**>( &m_as->buffer ),
                    gas_buffer_sizes.outputSizeInBytes
                    ) );
        }
    }

    m_as->buffer_size = gas_buffer_sizes.outputSizeInBytes;
    m_as->n_elements = 1;

    RM_OPTIX_CHECK( optixAccelBuild(
                m_ctx->ref(),
                m_stream->handle(),                  // CUDA stream
                &accel_options,
                &triangle_input,
                1,                  // num build inputs
                d_temp_buffer_gas,
                gas_buffer_sizes.tempSizeInBytes,
                m_as->buffer,
                gas_buffer_sizes.outputSizeInBytes,
                &m_as->handle,
                nullptr,            // emitted property list
                0                   // num emitted properties
                ) );

    RM_CUDA_CHECK( cudaFree( reinterpret_cast<void*>( d_temp_buffer_gas ) ) );
}

void OptixMesh::computeFaceNormals()
{
    if(face_normals.size() != faces.size())
    {
        face_normals.resize(faces.size());
    }
    rmagine::computeFaceNormals(vertices, faces, face_normals);
}


const CUdeviceptr* OptixMesh::getVertexBuffer() const
{
    return &m_vertices_ref;
}

CUdeviceptr OptixMesh::getFaceBuffer()
{
    return reinterpret_cast<CUdeviceptr>(faces.raw());
}

OptixMeshPtr make_optix_mesh(
    const aiMesh* amesh,
    OptixContextPtr context)
{
    OptixMeshPtr ret = std::make_shared<OptixMesh>(context);

    const aiVector3D* ai_vertices = amesh->mVertices;
    unsigned int num_vertices = amesh->mNumVertices;
    const aiFace* ai_faces = amesh->mFaces;
    unsigned int num_faces = amesh->mNumFaces;

    Memory<Point, RAM> vertices_cpu(num_vertices);
    Memory<Face, RAM> faces_cpu(num_faces);
    Memory<Vector, RAM> face_normals_cpu(num_faces);
    
    // convert
    for(size_t i=0; i<num_vertices; i++)
    {
        vertices_cpu[i] = {
                ai_vertices[i].x,
                ai_vertices[i].y,
                ai_vertices[i].z};
            }
    ret->vertices = vertices_cpu;

    for(size_t i=0; i<num_faces; i++)
    {
        faces_cpu[i].v0 = ai_faces[i].mIndices[0];
        faces_cpu[i].v1 = ai_faces[i].mIndices[1];
        faces_cpu[i].v2 = ai_faces[i].mIndices[2];
    }
    ret->faces = faces_cpu;

    ret->computeFaceNormals();

    if(amesh->HasNormals())
    {
        // has vertex normals
        Memory<Vector, RAM> vertex_normals_cpu(num_faces);
        vertex_normals_cpu.resize(num_vertices);
        for(size_t i=0; i<num_vertices; i++)
        {
            vertex_normals_cpu[i] = convert(amesh->mNormals[i]);
        }
        // upload
        ret->vertex_normals = vertex_normals_cpu;
    }

    if(amesh->HasTextureCoords(0))
    {
        Memory<Vector2, RAM> vertex_uvs_cpu(num_vertices);
        for(size_t i=0; i<num_vertices; i++)
        {
            vertex_uvs_cpu[i] = {amesh->mTextureCoords[0][i].x, amesh->mTextureCoords[0][i].y};
        }
        // upload
        ret->vertex_uvs = vertex_uvs_cpu;
    }

    ret->apply();

    return ret;
}

} // namespace rmagine