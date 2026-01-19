#include "rmagine/map/optix/OptixScene.hpp"
#include "rmagine/map/optix/OptixNode.hpp"
#include "rmagine/map/optix/OptixMaterial.hpp"
#include "rmagine/map/optix/OptixMesh.hpp"
#include "rmagine/map/optix/OptixTexture.hpp"
#include "rmagine/map/optix/OptixAccelerationStructure.hpp"
#include "rmagine/map/optix/OptixSceneEventReceiver.hpp"

#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/util/optix/OptixDebug.hpp>

#include <optix_stubs.h>

#include <rmagine/math/assimp_conversions.h>
#include <rmagine/util/assimp/helper.h>
#include <rmagine/math/linalg.h>

// use own lib instead
#include "rmagine/util/optix/OptixUtil.hpp"
#include "rmagine/util/optix/OptixSbtRecord.hpp"
#include "rmagine/util/optix/OptixData.hpp"

namespace rmagine
{

OptixScene::OptixScene(OptixContextPtr context)
:OptixEntity(context)
{
    m_root = std::make_shared<OptixNode>(context);
}
OptixScene::~OptixScene()
{

}

void OptixScene::buildIAS()
{
    std::vector<OptixNodePtr> nodes;
    m_root->get_renderables(nodes);
    const size_t n_instances = nodes.size();
    // fill m_hitgroup_data
    Memory<OptixInstance, RAM> inst_h(n_instances);
    for(int idx = 0; idx < n_instances; idx++)
    {
        OptixNodePtr node = nodes[idx];
        OptixInstance inst = {};
        const Matrix4x4& M = node->get_world_matrix();
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
        inst.traversableHandle = node->get_mesh()->as()->handle;
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

void OptixScene::commit()
{
    for (auto mesh : m_meshes)
        mesh->buildGAS();
    buildIAS();
}

OptixMeshPtr OptixScene::get_mesh(const std::string& name)
{
    for (OptixMeshPtr mesh : m_meshes)
        if (mesh->name == name)
            return mesh;
    return OptixMeshPtr();
}

void make_optix_tree(const aiNode* ai_node, OptixScenePtr scene, OptixNodePtr node)
{
    Matrix4x4 M = convert(ai_node->mTransformation);
    Transform T;
    Vector3 scale;
    decompose(M, T, scale);
    node->setTransform(T);
    node->setScale(scale);
    if(ai_node->mNumMeshes > 0)
    {
        node->set_mesh(scene->get_mesh(ai_node->mMeshes[0]));
    }
    for(size_t i=0; i<ai_node->mNumChildren; i++)
    {
        OptixNodePtr child = node->add_node();
        make_optix_tree(ai_node->mChildren[i], scene, child);
    }
}

OptixScenePtr make_optix_scene(
    const aiScene* ascene,
    OptixContextPtr context)
{
    OptixScenePtr scene = std::make_shared<OptixScene>(context);
    for(size_t i=0; i<ascene->mNumTextures; i++)
    {
        const aiTexture* atexture = ascene->mTextures[i];
        OptixTexturePtr texture = std::make_shared<OptixTexture>(context);
        texture->init(atexture->mWidth, atexture->mHeight, (uint32_t*)atexture->pcData);
        scene->add_texture(texture);
    }
    for(size_t i=0; i<ascene->mNumMaterials; i++)
    {
        const aiMaterial* amaterial = ascene->mMaterials[i];
        OptixMaterialPtr material = std::make_shared<OptixMaterial>(context);
        aiTextureType types[4] = {aiTextureType::aiTextureType_DIFFUSE, aiTextureType::aiTextureType_NORMALS, aiTextureType::aiTextureType_METALNESS, aiTextureType::aiTextureType_DIFFUSE_ROUGHNESS};
        for(size_t j=0; j<4; j++)
        {
            aiString str;
            if(amaterial->GetTextureCount(types[j]))
            {
                amaterial->GetTexture(types[j], 0, &str);
                std::cout << amaterial->GetName().C_Str() << str.C_Str() << std::endl;
            }
        }
        scene->add_material(material);
    }
    for(size_t i=0; i<ascene->mNumMeshes; i++)
    {
        const aiMesh* amesh = ascene->mMeshes[i];
        if(amesh->mPrimitiveTypes & aiPrimitiveType_TRIANGLE)
        {
            // triangle mesh
            OptixMeshPtr mesh = make_optix_mesh(amesh, context);
            // OptixMeshPtr mesh = std::make_shared<OptixMesh>(amesh);
            mesh->commit();
            scene->add_mesh(mesh);
            // std::cout << "Mesh " << i << "(" << mesh->name << ") added." << std::endl;
        } else {
            std::cout << "[ make_optix_scene(aiScene) ] WARNING: Could not construct geometry " << i << " prim type " << amesh->mPrimitiveTypes << " not supported yet. Skipping." << std::endl;
        }
    }
    make_optix_tree(ascene->mRootNode, scene, scene->root());
    return scene;
}

} // namespace rmagine