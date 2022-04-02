/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "graph/node.h"
#include "scene/attribute.h"
#include "scene/procedural.h"
#include "util/set.h"
#include "util/transform.h"
#include "util/vector.h"

#ifdef WITH_USD

CCL_NAMESPACE_BEGIN

class USDProcedural : public Procedural {
 public:
  NODE_DECLARE

  /* The file path to the USD stage */
  NODE_SOCKET_API(ustring, filepath)

  USDProcedural();
  ~USDProcedural();

  void generate(Scene *scene, Progress &progress);
};

CCL_NAMESPACE_END

#endif
