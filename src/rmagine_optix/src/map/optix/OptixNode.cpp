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

Memory<LiDARModel, RAM> create_lidar_model()
{
    Memory<LiDARModel, RAM> model(1);
    model->theta.min = -M_PI;
    model->theta.inc = 0.4 * M_PI / 180.0;
    model->theta.size = 900;

    model->phi.min = -15.0 * M_PI / 180.0;
    model->phi.inc = 2.0 * M_PI / 180.0;
    model->phi.size = 16;

    model->range.min = 0.1;
    model->range.max = 130.0;
    return model;
}

Memory<CameraModel, RAM> create_camera_model()
{
    Memory<CameraModel, RAM> model(1);
    model->width = 640;
    model->height = 480;
    model->c[0] = 319.5; // ~ half of width
    model->c[1] = 239.5; // ~ half of height
    model->f[0] = 525;
    model->f[1] = 525;
    model->range.min = 0.0;
    model->range.max = 100.0;
    return model;
}

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

OptixNodePtr OptixNode::add_node()
{
    OptixNodePtr node = std::make_shared<OptixNode>(m_ctx);
    add_child(node);
    return node;
}

OptixCameraPtr OptixNode::add_camera()
{
    OptixCameraPtr node = std::make_shared<OptixCamera>(m_ctx);
    add_child(node);
    return node;
}

OptixLidarPtr OptixNode::add_lidar()
{
    OptixLidarPtr node = std::make_shared<OptixLidar>(m_ctx);
    add_child(node);
    return node;
}

void OptixNode::get_renderables(std::vector<OptixNodePtr>& nodes)
{
    m_world = matrix();
    if (m_parent)
        m_world = m_parent->get_world_matrix() * m_world;
    if (m_mesh)
        nodes.emplace_back(this_shared<OptixNode>());
    for (OptixNodePtr child : m_children)
        child->get_renderables(nodes);
}

OptixNodePtr OptixNode::get_node(const std::string& path)
{
    size_t start = 1;
    size_t end = path.find("/",start);
    if (end == std::string::npos)
        end = path.length();
    if (end <= start)
        return this_shared<OptixNode>();
    std::string name = path.substr(start, end-start);
    for (OptixNodePtr child : m_children)
    {
        if (child->name == name)
        {
            OptixNodePtr ret = child->get_node(path.substr(end));
            if (ret)
                return ret;
        }
    }
    return OptixNodePtr();
}

void OptixNode::set_mesh(OptixMeshPtr mesh)
{
    m_mesh = mesh;
    m_S = mesh->local_scale;
}

OptixCamera::OptixCamera(OptixContextPtr context)
:OptixNode(context)
{
    // std::cout << "[OptixGeometry::OptixGeometry()] constructed." << std::endl;
}

OptixCamera::~OptixCamera()
{
    // std::cout << "[OptixGeometry::~OptixGeometry()] destroyed." << std::endl;
}

void OptixCamera::create_sim(OptixMapPtr map)
{
    sim = std::make_shared<PinholeSimulatorOptix>(map);
    model = create_camera_model();
    sim->setModel(model);
}

OptixLidar::OptixLidar(OptixContextPtr context)
:OptixNode(context)
{
    // std::cout << "[OptixGeometry::OptixGeometry()] constructed." << std::endl;
}

OptixLidar::~OptixLidar()
{
    // std::cout << "[OptixGeometry::~OptixGeometry()] destroyed." << std::endl;
}

void OptixLidar::create_sim(OptixMapPtr map)
{
    sim = std::make_shared<SphereSimulatorOptix>(map);
    model = create_lidar_model();
    sim->setModel(model);
}

} // namespace rmagine