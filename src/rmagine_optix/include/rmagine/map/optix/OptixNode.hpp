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
 * @brief OptixGeometry
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMAGINE_MAP_OPTIX_NODE_HPP
#define RMAGINE_MAP_OPTIX_NODE_HPP

#include <memory>
#include <rmagine/math/types.h>
#include <rmagine/util/optix/OptixContext.hpp>
#include <rmagine/simulation/PinholeSimulatorOptix.hpp>
#include <rmagine/simulation/SphereSimulatorOptix.hpp>
#include <vector>
#include <unordered_set>


#include "optix_definitions.h"
#include "OptixEntity.hpp"
#include "OptixTransformable.hpp"

namespace rmagine
{

class OptixNode
: public OptixEntity
, public OptixTransformable
{
public:
    OptixNode(OptixContextPtr context = optix_default_context());
    virtual ~OptixNode();
    virtual void apply(){}
    OptixNodePtr add_node();
    OptixCameraPtr add_camera();
    OptixLidarPtr add_lidar();
    void add_child(OptixNodePtr child)
    {
        child->set_parent(this_shared<OptixNode>());
        m_children.emplace_back(child);
    }
    void get_renderables(std::vector<OptixNodePtr>& nodes);
    void set_mesh(OptixMeshPtr mesh);
    void set_parent(OptixNodePtr parent){m_parent = parent;}
    const Matrix4x4& get_world_matrix() {return m_world;}
    OptixMeshPtr get_mesh(){return m_mesh;}
    OptixNodePtr get_node(const std::string& path);
protected:
    OptixMeshPtr m_mesh;
    OptixNodePtr m_parent;
    Matrix4x4 m_world;
    std::vector<OptixNodePtr> m_children;
};

class OptixCamera : public OptixNode
{
public:
    OptixCamera(OptixContextPtr context = optix_default_context());
    virtual ~OptixCamera();
    void create_sim(OptixMapPtr map);
    template<typename BundleT>
    BundleT get_data()
    {
        return sim->simulate<BundleT>(m_T);
    }
protected:
    PinholeSimulatorOptixPtr sim;
    Memory<CameraModel, RAM> model;
};

class OptixLidar : public OptixNode
{
public:
    OptixLidar(OptixContextPtr context = optix_default_context());
    virtual ~OptixLidar();
    void create_sim(OptixMapPtr map);
    template<typename BundleT>
    BundleT get_data()
    {
        return sim->simulate<BundleT>(m_T);
    }
protected:
    SphereSimulatorOptixPtr sim;
    Memory<LiDARModel, RAM> model;
};

} // namespace rmagine

#endif // RMAGINE_MAP_OPTIX_NODE_HPP

