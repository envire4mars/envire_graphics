/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "EnvireGraphViz.h"

#include <stdlib.h>
#include <algorithm>
#include <cassert>
#include <sstream>

#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

#include <mars/interfaces/graphics/GraphicsManagerInterface.h>

#include <base/TransformWithCovariance.hpp>

#include <envire_core/graph/EnvireGraph.hpp>

#include <mars/plugins/envire_managers/EnvireDefs.hpp>
#include <mars/plugins/envire_managers/EnvireStorageManager.hpp>




using namespace mars::plugins::envire_graphics;
using namespace mars::plugins::envire_managers;

using vertex_descriptor = envire::core::GraphTraits::vertex_descriptor;

//LOG_DEBUG with stringstream for easy conversion
//#define LOG_DEBUG_S(...) \
//  std::stringstream ss; \
//  ss << __VA_ARGS__; \
//  LOG_DEBUG(ss.str());

EnvireGraphViz::EnvireGraphViz(lib_manager::LibManager *theManager)
  : MarsPluginTemplate(theManager, "EnvireGraphViz"), GraphEventDispatcher()
{
    // FIX: take originId from the node manager
    // change the name of origin id from center into the world
    updateTree(SIM_CENTER_FRAME_NAME);

    assert(control->storage->getGraph() != nullptr);

    GraphEventDispatcher::subscribe(control->storage->getGraph().get());

    GraphItemEventDispatcher<envire::core::Item<::smurf::Joint>>::subscribe(control->storage->getGraph().get());
    GraphItemEventDispatcher<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>::subscribe(control->storage->getGraph().get());
    GraphItemEventDispatcher<envire::core::Item<maps::grid::MLSMapPrecalculated>>::subscribe(control->storage->getGraph().get());

    if (!control->storage->getGraph()->containsFrame(SIM_CENTER_FRAME_NAME))
    {
      throw std::runtime_error("Graph has no Center Frame");
    }
}

void EnvireGraphViz::init() {}

void EnvireGraphViz::reset() {
}

void EnvireGraphViz::setPos(const envire::core::FrameId& frame, mars::interfaces::NodeData& node)
{
    envire::core::Transform fromOrigin;
    if(SIM_CENTER_FRAME_NAME.compare(frame) == 0) {
      //this special case happens when the graph only contains one frame
      //and items are added to that frame. In that case asking the graph
      //for the transformation would cause an exception
      fromOrigin.setTransform(base::TransformWithCovariance::Identity());
    }
    else {
      fromOrigin = control->storage->getGraph()->getTransform(SIM_CENTER_FRAME_NAME, frame);
    }
    node.pos = fromOrigin.transform.translation;
    node.rot = fromOrigin.transform.orientation;
}

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>& e)
{

    // FIX: do we need? control->sim->sceneHasChanged(false);
    std::shared_ptr<mars::sim::SimNode> simNode = e.item->getData();

    int visual_rep = simNode->getVisualRep();

    // ----- Set Visual Representation
    mars::interfaces::NodeData nodeData = simNode->getSNode();
    //std::ostringstream log;

    mars::interfaces::NodeId id = control->graphics->addDrawObject(nodeData, visual_rep & 1);
    if(id) {
        simNode->setGraphicsID(id);
        uuidToGraphicsId[e.item->getID()] = id;
    }

    /// ---- Set Physical Representation -> collision objects
    if(nodeData.noPhysical == false && nodeData.physicMode != mars::interfaces::NODE_TYPE_TERRAIN) {
        mars::interfaces::NodeData physicalRep;
        physicalRep = nodeData;
        physicalRep.material = nodeData.material;
        physicalRep.material.exists = 1;
        physicalRep.material.transparency = 0.3;
        physicalRep.material.name += "_trans";
        physicalRep.visual_offset_pos = mars::utils::Vector(0.0, 0.0, 0.0);
        physicalRep.visual_offset_rot = mars::utils::Quaternion::Identity();
        physicalRep.visual_size = mars::utils::Vector(0.0, 0.0, 0.0);
        physicalRep.map["sharedDrawID"] = 0lu;
        physicalRep.map["visualType"] = mars::interfaces::NodeData::toString(nodeData.physicMode);

        if(nodeData.physicMode != mars::interfaces::NODE_TYPE_MESH) {
            physicalRep.filename = "PRIMITIVE";
            //physicalRep.filename = nodeS->filename;
            if(nodeData.physicMode > 0 && nodeData.physicMode < mars::interfaces::NUMBER_OF_NODE_TYPES){
                physicalRep.origName = mars::interfaces::NodeData::toString(nodeData.physicMode);
            }
        }

        id = control->graphics->addDrawObject(physicalRep, visual_rep & 2);
        if(id) {
            simNode->setGraphicsID2(id);
            uuidToGraphicsId2[e.item->getID()] = id;
        }
    }
    //LOG_DEBUG(log.str().c_str());
}

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Joint>>& e)
{
    if (viewJoints)
    {
        const envire::core::FrameId source = e.item->getData().getSourceFrame().getName();
        const envire::core::FrameId target = e.item->getData().getTargetFrame().getName();

        const envire::core::Transform tf = control->storage->getGraph()->getTransform(source, target);
        const double length = tf.transform.translation.norm();
        base::Vector3d extents(0.01, length, 0);

        mars::interfaces::NodeData node;
        node.initPrimitive(mars::interfaces::NODE_TYPE_CYLINDER, extents, 0.00001); //mass is zero because it doesnt matter for visual representation
        node.material.emissionFront = mars::utils::Color(0.0, 1.0, 0.0, 1.0);
        node.material.transparency = 0.5;

        const envire::core::Transform originToSource = control->storage->getGraph()->getTransform(SIM_CENTER_FRAME_NAME, source);
        const envire::core::Transform originToTarget = control->storage->getGraph()->getTransform(SIM_CENTER_FRAME_NAME, target);
        node.pos = (originToSource.transform.translation + originToTarget.transform.translation) / 2.0;
        node.rot = e.item->getData().getParentToJointOrigin().rotation();

        uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
    }
}

void EnvireGraphViz::updateMLSVis()
{
  envire::core::Transform simTf = control->storage->getGraph()->getTransform(SIM_CENTER_FRAME_NAME, mlsFrameName);
  osg::Vec3d vector;
  osg::Quat quat;

  vector.x() = simTf.transform.translation.x();
  vector.y() = simTf.transform.translation.y();
  vector.z() = simTf.transform.translation.z();

  quat.x() = simTf.transform.orientation.x();
  quat.y() = simTf.transform.orientation.y();
  quat.z() = simTf.transform.orientation.z();
  quat.w() = simTf.transform.orientation.w();

  visTf->setPosition(vector);
  visTf->setAttitude(quat);
}

void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<maps::grid::MLSMapPrecalculated>>& e)
{
  LOG_DEBUG("[EnvireGraphViz::itemAdded<MLSMapPrecalculated>] Added an MLS to the graph, let's visualize it");

  //if (isConnectedSurface() == false)
  //{
  //  std::cout << "EnvireGraphViz: for better mls visualisation we turned on connected surface" << std::endl;
  //  //setConnectedSurfaceLOD(true);
  //  setConnectedSurface(true);
  //}
    if (visTf.valid())
        control->graphics->removeOSGNode(visTf);


    setShowMapExtents(true);
    //setConnectedSurface(true);
    //setShowNormals(true);

  maps::grid::MLSMapPrecalculated map = e.item->getData();
  osgNode = createMainNode(); // vizkit3d Protected
  updateData(map);
  updateMainNode(osgNode);// vizkit3d Protected
  osgGroup = getVizNode();
  mlsFrameName = e.frame;
  visTf = new osg::PositionAttitudeTransform();
  visTf->addChild(osgNode);
  updateMLSVis();

  if (!osgGroup){ LOG_DEBUG("[EnvireGraphViz::itemAdded<MLSMapPrecalculated>] The generated osgGroup is null");}
  else {LOG_DEBUG("[EnvireGraphViz::itemAdded<MLSMapPrecalculated>] The OSG group is not null");}
  control->graphics->addOSGNode(visTf);


  // We don't get any id back from addOSGNode, so I guess we don't need the following:
  //uuidToGraphicsId[e.item->getID()] = control->graphics->addDrawObject(node); //remeber graphics handle
}

void EnvireGraphViz::itemRemoved(const envire::core::TypedItemAddedEvent<envire::core::Item<maps::grid::MLSMapPrecalculated>>& e) {
    std::cout << "ITEM REMOVED" << std::endl;
    if (visTf.valid())
        control->graphics->removeOSGNode(visTf);
}

void EnvireGraphViz::itemAdded(const envire::core::ItemAddedEvent& e)
{
  /*//FIXME replace with specific itemAddedEvent for PhysicsConfigMapItem
  boost::shared_ptr<PhysicsConfigMapItem> pItem;
  if(pItem = boost::dynamic_pointer_cast<PhysicsConfigMapItem>(e.item))
  {
    NodeData node1;
    node1.fromConfigMap(&pItem->getData(), "");

  // MLS objects are visualized using their own osg visual (see void EnvireGraphViz::itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<maps::grid::MLSMapKalman>>& e))
  if(node1.physicMode != NODE_TYPE_MLS) //TODO: implement a visualization for mls
  {
    //assert that this item has not been added before
    assert(uuidToGraphicsId.find(pItem->getID()) == uuidToGraphicsId.end());
    try
    {
      NodeData node;
      if(node.fromConfigMap(&pItem->getData(), ""))
      {
        // TODO Fix: The emission Front is lost when going to config map and back
        node.material.emissionFront = mars::utils::Color(1.0, 1.0, 1.0, 1.0);
        node.material.transparency = 0.5;
        setPos(e.frame, node);
        uuidToGraphicsId[pItem->getID()] = control->graphics->addDrawObject(node);
      }
    }
    catch(const UnknownTransformException& ex)
    {
      LOG_ERROR(ex.what());
    }
  }
  }*/
}


void EnvireGraphViz::setNodeDataMaterial(mars::interfaces::NodeData& nodeData, urdf::MaterialSharedPtr material) const
{
  nodeData.material.texturename = material->texture_filename;
  nodeData.material.diffuseFront = mars::utils::Color(material->color.r, material->color.g,
                                                      material->color.b, material->color.a);
}


void EnvireGraphViz::update(mars::interfaces::sReal time_ms) {
  const float timeBetweenFramesMs = 1000.0 / visualUpdateRateFps;
  timeSinceLastUpdateMs += time_ms;

  if(timeSinceLastUpdateMs >= timeBetweenFramesMs)
  {
    updateVisuals();
    timeSinceLastUpdateMs = 0;
  }
}

void EnvireGraphViz::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
}

void EnvireGraphViz::updateTree(const envire::core::FrameId& origin)
{
  const vertex_descriptor newOrigin = control->storage->getGraph()->vertex(origin);
  assert(newOrigin != control->storage->getGraph()->null_vertex());
  tree.clear();
  control->storage->getGraph()->getTree(newOrigin, true, &tree);
}


void EnvireGraphViz::updateVisuals()
{
  if (tree.hasRoot() == false)
    return;

  tree.visitBfs(tree.root, [&](envire::core::GraphTraits::vertex_descriptor vd,
                               envire::core::GraphTraits::vertex_descriptor parent)
  {
    updatePosition<envire::core::Item<std::shared_ptr<mars::sim::SimNode>>>(vd);
  });
}


/**Updates the drawing position of @p vertex */
template <class physicsType> void EnvireGraphViz::updatePosition(const vertex_descriptor vertex)
{
  const envire::core::FrameId& frameId = control->storage->getGraph()->getFrameId(vertex);
  base::Vector3d translation;
  base::Quaterniond orientation;
  if(SIM_CENTER_FRAME_NAME.compare(frameId) == 0)
  {
    translation << 0, 0, 0;
    orientation.setIdentity();
  }
  else
  {
    if(pathsFromOrigin.find(vertex) == pathsFromOrigin.end())
    {
      //this is an unknown vertex, find the path and store it
      pathsFromOrigin[vertex] = control->storage->getGraph()->getPath(SIM_CENTER_FRAME_NAME, frameId, true);
    }
    const envire::core::Transform tf = control->storage->getGraph()->getTransform(pathsFromOrigin[vertex]);
    translation = tf.transform.translation;
    orientation = tf.transform.orientation;
  }

  using Iterator = envire::core::EnvireGraph::ItemIterator<physicsType>;
  Iterator begin, end;
  boost::tie(begin, end) = control->storage->getGraph()->getItems<physicsType>(vertex);
  for(;begin != end; ++begin)
  {
    const physicsType& item = *begin;
    //others might use the same types as well, therefore check if if this is one of ours
    if(uuidToGraphicsId.find(item.getID()) != uuidToGraphicsId.end())
    {
      const int graphicsId = uuidToGraphicsId.at(item.getID());
      control->graphics->setDrawObjectPos(graphicsId, translation);
      control->graphics->setDrawObjectRot(graphicsId, orientation);

    }

    if(uuidToGraphicsId2.find(item.getID()) != uuidToGraphicsId2.end())
    {

      const int graphicsId2 = uuidToGraphicsId2.at(item.getID());
      control->graphics->setDrawObjectPos(graphicsId2, translation);
      control->graphics->setDrawObjectRot(graphicsId2, orientation);
    }
  }
}

DESTROY_LIB(mars::plugins::envire_graphics::EnvireGraphViz);
CREATE_LIB(mars::plugins::envire_graphics::EnvireGraphViz);