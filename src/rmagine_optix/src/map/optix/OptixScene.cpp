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

void OptixScene::commit()
{
    for (auto mesh : m_meshes)
        mesh->buildGAS();
    m_root->buildIAS();
    m_as = m_root->as();
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
        OptixNodePtr child = node->add_child();
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
    make_optix_tree(ascene->mRootNode, scene, scene->get_root());
    return scene;
}

} // namespace rmagine