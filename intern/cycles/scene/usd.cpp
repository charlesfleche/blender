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

#  include <pxr/base/gf/matrix4d.h>
#  include <pxr/base/plug/registry.h>
#  include <pxr/usd/usd/prim.h>
#  include <pxr/usd/usd/primRange.h>
#  include <pxr/usd/usd/stage.h>
#  include <pxr/usd/usdGeom/mesh.h>
#  include <pxr/usd/usdGeom/xformCache.h>

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
  static bool plugin_path_registered = false;
  if (plugin_path_registered) {
    return;
  }
  plugin_path_registered = true;
  // TODO: compute plugin path
  pxr::PlugRegistry::GetInstance().RegisterPlugins(
      "/home/charles/blender-git/lib/linux_x86_64/usd/lib/usd");
}

USDProcedural::~USDProcedural()
{
}
#  if 0
template<typename tUsd> struct Trait {
  typedef void CyclesType;
};

template<> struct Trait<GfVec3f> {
  typedef float3 CyclesType;
};

template<typename tIn, typename tOut> tOut convert(const tIn &, tOut &);

template<typename tIn, typename tOut, typename = std::enable_if_t<std::is_same<tIn, tOut>::value>>
void convert(const tIn &in, tOut &out)
{
  return out = in;
}

template<typename tIn, typename tOut> void convert(const tIn &in, tOut &out)
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
#  endif

static void convert(const GfVec3f &in, float3 &out)
{
  for (size_t i = 0; i < GfVec3f::dimension; ++i) {
    out[i] = in[i];
  }
}

static void convert(const GfMatrix4d &in, Transform &out)
{
  const auto data = in.data();

  //  out.x[0] = static_cast<float>(data[0]);
  //  out.x[1] = static_cast<float>(data[1]);
  //  out.x[2] = static_cast<float>(data[2]);
  //  out.x[3] = static_cast<float>(data[3]);

  //  out.y[0] = static_cast<float>(data[4]);
  //  out.y[1] = static_cast<float>(data[5]);
  //  out.y[2] = static_cast<float>(data[6]);
  //  out.y[3] = static_cast<float>(data[7]);

  //  out.z[0] = static_cast<float>(data[8]);
  //  out.z[1] = static_cast<float>(data[9]);
  //  out.z[2] = static_cast<float>(data[10]);
  //  out.z[3] = static_cast<float>(data[11]);

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

static void convert(const VtArray<GfVec3f> &in, array<float3> &out)
{
  out.resize(in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    const auto &in_vec = in[i];
    auto &out_vec = out[i];
    convert(in[i], out[i]);
    auto test = out;
  }
}

static void convert(const VtArray<int> &in, array<int> &out)
{
  // TODO: memcopy
  out.resize(in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    out[i] = in[i];
  }
}
#  if 1
static void read(const UsdGeomMesh &mesh, Mesh *geometry)
{
  // TODO: handle other mesh types
  geometry->set_subdivision_type(Mesh::SubdivisionType::SUBDIVISION_CATMULL_CLARK);

  // Verts

  VtArray<int> fvcs;
  VtArray<int> fvis;
  VtArray<GfVec3f> points;
  if (!mesh.GetPointsAttr().Get(&points) || !mesh.GetFaceVertexIndicesAttr().Get(&fvis) ||
      !mesh.GetFaceVertexCountsAttr().Get(&fvcs)) {
    // TODO: log error
    return;
  }

  array<float3> verts;
  convert(points, verts);

  geometry->set_verts(verts);

  // Triangles

  array<int> triangles;
  int face_index_offset = 0;
  for (auto c : fvcs) {
    // TODO: handle log face with less than 3 points
    auto first_point_index = fvis[face_index_offset];
    for (int second_point_offset = 1, third_point_offset = 2; third_point_offset < c;
         ++second_point_offset, ++third_point_offset) {
      auto a = fvis[face_index_offset];
      auto b = fvis[face_index_offset + second_point_offset];
      auto c = fvis[face_index_offset + third_point_offset];
      triangles.push_back_slow(a);
      triangles.push_back_slow(b);
      triangles.push_back_slow(c);
    }
    face_index_offset += c;
  }
  geometry->set_triangles(triangles);
}
#  else

static void read(const UsdGeomMesh &, Mesh *geometry)
{
  array<float3> verts;
  verts.push_back_slow(make_float3(0, 0, 0));
  verts.push_back_slow(make_float3(1, 0, 0));
  verts.push_back_slow(make_float3(1, 1, 0));
  geometry->set_verts(verts);

  array<int> triangles;
  triangles.push_back_slow(0);
  triangles.push_back_slow(1);
  triangles.push_back_slow(2);
  geometry->set_triangles(triangles);
}
#  endif

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
  //    object->set_owner(this); // TODO: where is the Generator stored ?
  object->set_geometry(geometry);
  object->name = geometry->name;

  const auto mat = cache.GetLocalToWorldTransform(prim);
  Transform tfm;
  convert(mat, tfm);

  object->set_tfm(tfm);  // TODO: probably wrong orientation

  return object;
}

void USDProcedural::generate(Scene *scene, Progress &progress)
{
  auto plugins = PlugRegistry::GetInstance().GetAllPlugins();
  auto stage = UsdStage::Open(filepath.c_str());

  if (!stage) {
    // TODO: log error
    return;
  }

  UsdGeomXformCache cache;
  for (const auto &prim : stage->Traverse()) {
    generate_node(scene, progress, cache, prim);
  }

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
