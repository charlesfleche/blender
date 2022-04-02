/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "scene/usd.h"

#include "scene/camera.h"
#include "scene/curves.h"
#include "scene/mesh.h"
#include "scene/object.h"
#include "scene/pointcloud.h"
#include "scene/scene.h"
#include "scene/shader.h"

#include "util/foreach.h"
#include "util/log.h"
#include "util/progress.h"
#include "util/transform.h"
#include "util/vector.h"

#include <type_traits>

#ifdef WITH_USD

#  include <pxr/usd/usd/prim.h>
#  include <pxr/usd/usd/primRange.h>
#  include <pxr/usd/usd/stage.h>
#  include <pxr/usd/usdGeom/mesh.h>

PXR_NAMESPACE_USING_DIRECTIVE

CCL_NAMESPACE_BEGIN

NODE_DEFINE(USDProcedural)
{
  NodeType *type = NodeType::add("usd", create);

  SOCKET_STRING(filepath, "Filename", ustring());

  return type;
}

USDProcedural::USDProcedural() : Procedural(get_node_type())
{
}

USDProcedural::~USDProcedural()
{
}

template<typename tIn, typename tOut> tOut convert(const tIn &);

template<typename tIn, typename tOut, typename = std::enable_if_t<std::is_same<tIn, tOut>::value>>
int convert(const int &val)
{
  return val;
}

template<> float3 convert(const GfVec3f &vec)
{
  return make_float3(vec[0], vec[1], vec[2]);
}

template<typename tIn, typename tOut> void convert(const VtArray<tIn> &vt, array<tOut> &arr)
{
  arr.clear();
  for (const auto &val : vt) {
    arr.push_back_slow(convert<tIn, tOut>(val));
  }
}

void USDProcedural::generate(Scene *scene, Progress &progress)
{
  auto stage = UsdStage::Open(filepath.c_str());

  //  for (auto prim : stage->Traverse()) {
  //    UsdGeomMesh usd_mesh(prim);
  //    if (usd_mesh) {
  //      VtArray<GfVec3f> points;
  //      if (!usd_mesh.GetPointsAttr().Get(&points))
  //        continue;

  //      VtArray<int> face_vertex_indices;
  //      if (!usd_mesh.GetFaceVertexIndicesAttr().Get(&face_vertex_indices))
  //        continue;

  //      VtArray<int> face_vertex_counts;
  //      if (!usd_mesh.GetFaceVertexCountsAttr().Get(&face_vertex_counts))
  //        continue;

  //      auto mesh = scene->create_node<Mesh>();

  //      array<float3> P_array;
  //      convert(points, P_array);
  //      mesh->set_verts(P_array);

  //      array<int> verts;
  //      //      convert(face_vertex_indices, verts);

  //      array<int> nverts;
  //      //      convert(face_vertex_counts, nverts);

  //      auto object = scene->create_node<Object>();
  //      object->set_geometry(mesh);
  //    }
  //  }
}

CCL_NAMESPACE_END

#endif
