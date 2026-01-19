/*
 * Copyright (c) 2022, University Osnabrück
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University Osnabrück nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL University Osnabrück BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * 
 * @brief OptixScene
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMAGINE_MAP_OPTIX_SCENE_HPP
#define RMAGINE_MAP_OPTIX_SCENE_HPP

#include <rmagine/util/optix/OptixContext.hpp>
#include <rmagine/util/IDGen.hpp>

#include "optix_definitions.h"
#include "optix_sbt.h"

#include "OptixEntity.hpp"
#include <vector>
#include <map>
#include <optional>

#include <assimp/scene.h>

#include <rmagine/types/MemoryCuda.hpp>
#include <unordered_set>


namespace rmagine
{

struct OptixSceneCommitResult
{
    bool depth_changed = false;
    bool sbt_size_changed = false;
};

class OptixScene
: public OptixEntity
{
public:
    OptixScene(OptixContextPtr context = optix_default_context());
    virtual ~OptixScene();
    void add_mesh(OptixMeshPtr mesh) {m_meshes.emplace_back(mesh);}
    void add_material(OptixMaterialPtr material) {m_materials.emplace_back(material);}
    void add_texture(OptixTexturePtr texture) {m_textures.emplace_back(texture);}
    void buildIAS();
    void commit();
    int num_meshes(){return m_meshes.size();}
    OptixMeshPtr get_mesh(int index){return m_meshes[index];}
    OptixMeshPtr get_mesh(const std::string& name);
    OptixNodePtr root(){return m_root;}
protected:
    std::vector<OptixMeshPtr> m_meshes;
    std::vector<OptixMaterialPtr> m_materials;
    std::vector<OptixTexturePtr> m_textures;
    OptixNodePtr m_root;
};

OptixScenePtr make_optix_scene(
    const aiScene* ascene, 
    OptixContextPtr context = optix_default_context());

} // namespace rmagine

#endif // RMAGINE_MAP_OPTIX_SCENE_HPP