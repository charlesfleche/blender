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
#  include <pxr/base/arch/fileSystem.h>
#  include <pxr/base/arch/systemInfo.h>
#  include <pxr/base/gf/matrix4d.h>
#  include <pxr/base/gf/rotation.h>
#  include <pxr/base/plug/registry.h>
#  include <pxr/usd/usd/prim.h>
#  include <pxr/usd/usd/primRange.h>
#  include <pxr/usd/usd/stage.h>
#  include <pxr/usd/usdGeom/mesh.h>
#  include <pxr/usd/usdGeom/metrics.h>
#  include <pxr/usd/usdGeom/xformCache.h>

PXR_NAMESPACE_USING_DIRECTIVE

CCL_NAMESPACE_BEGIN

NODE_DEFINE(USDProcedural)
{
  NodeType *type = NodeType::add("usd", create);

  SOCKET_STRING(filepath, "Filename", ustring());

  return type;
}

static const Transform &usd_to_cycles_axis_conversion_transform(const TfRefPtr<UsdStage> &stage)
{
  // Make and cache the axis transforms

  static bool initialized = false;
  static Transform from_usd_right_handed_y_up;
  static Transform from_usd_right_handed_z_up;
  static Transform from_usd_right_handed_unknown_up;

  if (!initialized) {
    const auto xaxis = make_float3(1.0, 0.0, 0.0);
    const auto scl = transform_scale(1.0, 1.0, -1.0);
    const auto rot = transform_rotate(static_cast<float>(-M_PI_2), xaxis);

    from_usd_right_handed_y_up = scl;
    from_usd_right_handed_z_up = scl * rot;
    from_usd_right_handed_unknown_up = transform_identity();

    initialized = true;
  }

  // Select the transforms depending on the UsdStage up axis

  auto upaxis = UsdGeomGetStageUpAxis(stage);

  if (upaxis != "Y" && upaxis != "Z") {
    upaxis = UsdGeomGetFallbackUpAxis();
  }

  if (upaxis == "Y") {
    return from_usd_right_handed_y_up;
  }

  if (upaxis == "Z") {
    return from_usd_right_handed_z_up;
  }

  return from_usd_right_handed_unknown_up;
}

//
// USD to Cycles types conversions
//

static void convert(const GfVec3f &in, float3 &out)
{
  for (size_t i = 0; i < GfVec3f::dimension; ++i) {
    out[i] = in[i];
  }
}

static void convert(const GfMatrix4d &in, Transform &out)
{
  const auto data = in.data();

  out.x[0] = static_cast<float>(data[0]);
  out.x[1] = static_cast<float>(data[4]);
  out.x[2] = static_cast<float>(data[8]);
  out.x[3] = static_cast<float>(data[12]);

  out.y[0] = static_cast<float>(data[1]);
  out.y[1] = static_cast<float>(data[5]);
  out.y[2] = static_cast<float>(data[9]);
  out.y[3] = static_cast<float>(data[13]);

  out.z[0] = static_cast<float>(data[2]);
  out.z[1] = static_cast<float>(data[6]);
  out.z[2] = static_cast<float>(data[10]);
  out.z[3] = static_cast<float>(data[14]);
}

//
// Reading data from USD
//

static void read(const UsdGeomMesh &mesh, Mesh *geometry)
{
  // TODO: handle other mesh types
  geometry->set_subdivision_type(Mesh::SubdivisionType::SUBDIVISION_NONE);

  // Verts

  VtArray<int> fvcs;
  VtArray<int> fvis;
  VtArray<GfVec3f> points;
  if (!mesh.GetPointsAttr().Get(&points) || !mesh.GetFaceVertexIndicesAttr().Get(&fvis) ||
      !mesh.GetFaceVertexCountsAttr().Get(&fvcs)) {
    // TODO: log error
    return;
  }

  array<float3> verts(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    auto point = points[i];
    convert(point, verts[i]);
  }

  geometry->set_verts(verts);

  // Triangles

  array<int> triangles;
  int face_index_offset = 0;
  for (auto c : fvcs) {
    // TODO: handle log face with less than 3 points
    for (int second_point_offset = 1, third_point_offset = 2; third_point_offset < c;
         ++second_point_offset, ++third_point_offset) {
      triangles.push_back_slow(fvis[face_index_offset]);
      triangles.push_back_slow(fvis[face_index_offset + second_point_offset]);
      triangles.push_back_slow(fvis[face_index_offset + third_point_offset]);
    }
    face_index_offset += c;
  }
  geometry->set_triangles(triangles);
}

//
// Cycles Node factories
//

static auto *generate_node(Scene *scene, Progress &, const UsdGeomMesh &mesh)
{
  auto geometry = scene->create_node<Mesh>();
  geometry->name = mesh.GetPrim().GetPath().GetString();

  read(mesh, geometry);

  return geometry;
}

static Object *generate_node(Scene *scene,
                             Progress &progress,
                             UsdGeomXformCache &cache,
                             const Transform &axis_conversion_tfm,
                             const UsdPrim &prim)
{
  // TODO: assert valid mesh
  Geometry *geometry = nullptr;

  if (prim.IsA<UsdGeomMesh>()) {
    geometry = generate_node(scene, progress, UsdGeomMesh(prim));
  }

  if (!geometry) {
    return nullptr;
  }

  auto object = scene->create_node<Object>();
  object->set_geometry(geometry);
  object->name = geometry->name;

  const auto mat = cache.GetLocalToWorldTransform(prim);
  Transform tfm;
  convert(mat, tfm);

  object->set_tfm(axis_conversion_tfm * tfm);

  return object;
}

//
// USDProcedural
//

USDProcedural::USDProcedural() : Procedural(get_node_type())
{
  static bool is_initialized = false;
  if (is_initialized) {
    return;
  }
  is_initialized = true;

  // TODO: needs a more robust, cross-platform way to register USD plugins path

  const auto path = ArchNormPath(ArchGetExecutablePath() + "/../3.2/datafiles/usd");
  pxr::PlugRegistry::GetInstance().RegisterPlugins(path);
}

USDProcedural::~USDProcedural()
{
}

void USDProcedural::generate(Scene *scene, Progress &progress)
{
  auto plugins = PlugRegistry::GetInstance().GetAllPlugins();
  auto stage = UsdStage::Open(filepath.c_str());

  auto axis_conversion_tfm = usd_to_cycles_axis_conversion_transform(stage);

  if (!stage) {
    // TODO: log error
    return;
  }

  UsdGeomXformCache cache;
  for (const auto &prim : stage->Traverse()) {
    auto node = generate_node(scene, progress, cache, axis_conversion_tfm, prim);
    if (node) {
      node->set_owner(this);
    }
  }
}

CCL_NAMESPACE_END

#endif
