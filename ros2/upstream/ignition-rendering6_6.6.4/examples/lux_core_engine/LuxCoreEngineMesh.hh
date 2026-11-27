/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINEMESH_HH_
#define IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINEMESH_HH_

#include <ignition/common/Console.hh>

#include <luxcore/luxcore.h>

#include "ignition/rendering/base/BaseMesh.hh"

#include "LuxCoreEngineGeometry.hh"
#include "LuxCoreEngineRenderTypes.hh"

namespace ignition {
namespace rendering {
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

/// \brief LuxCore implementation of the mesh class
class LuxCoreEngineMesh : public BaseMesh<LuxCoreEngineGeometry>
{
  /// \brief Constructor
  public: LuxCoreEngineMesh();

  // Documentation inherited.
  public: bool HasParent() const;

  // Documentation inherited.
  public: VisualPtr Parent() const;

  // Documentation inherited.
  public: void SetMaterial(MaterialPtr _material, bool _unique = true) override;

  // Documentation inherited.
  public: void SetLocalPosition(double _x, double _y, double _z) override;

  // Documentation inherited.
  public: void SetLocalScale(double _x, double _y, double _z) override;

  // Documentation inherited.
  public: void SetLocalRotation(double _r, double _p, double _y) override;

  /// \brief Set mesh name
  /// \param[in] name of the mesh
  public: void SetName(std::string name);

  /// \brief Add sub meshes
  /// \param[in] _child pointer of the sub mesh
  public: void AddSubMesh(const LuxCoreEngineSubMeshPtr _child);

  // Documentation inherited.
  public: SubMeshStorePtr SubMeshes() const override;

  /// \brief Update mesh object Transformation
  private: void UpdateTransformation();

  /// \brief A list of submeshes
  protected: LuxCoreEngineSubMeshStorePtr subMeshes;

  /// \brief Translation matrix
  private: float translationMatrix[16];

  /// \brief Rotation matrix
  private: float rotationMatrix[16];

  /// \brief Scale matrix
  private: float scaleMatrix[16];

  /// \brief Previous transformation matrix
  private: float previousMatrix[16];
};

class LuxCoreEngineSubMesh : public BaseSubMesh<LuxCoreEngineObject>
{
  // Documentation inherited.
  public: void SetMaterialImpl(MaterialPtr _material) override;

  /// \brief Set sub mesh name
  /// \param[in] name name of the given sub mesh
  public: void SetName(std::string name);
};

} // namespace IGNITION_RENDERING_VERSION_NAMESPACE
} // namespace rendering
} // namespace ignition

#endif
