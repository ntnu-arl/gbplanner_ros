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
#include "LuxCoreEngineMesh.hh"
#include "LuxCoreEngineScene.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
std::vector<float> multiplyMatrix(float *a, float *b)
{
  std::vector<float> result(16);
  for (uint32_t x = 0; x < 4; x++)
  {
    for (uint32_t y = 0; y < 4; y++)
    {
      float sum = 0;
      for (uint32_t z = 0; z < 4; z++)
      {
        sum += a[x * 4 + z] * b[z * 4 + y];
      }
      result[x * 4 + y] = sum;
    }
  }

  return result;
}

//////////////////////////////////////////////////
std::vector<float> inverseMatrix(float *m)
{
  float result[16];
  std::vector<float> resultOut(16);

  result[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] -
              m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] -
              m[13] * m[7] * m[10];

  result[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] +
              m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] +
              m[12] * m[7] * m[10];

  result[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
              m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];

  result[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] +
               m[8] * m[5] * m[14] - m[8] * m[6] * m[13] -
               m[12] * m[5] * m[10] + m[12] * m[6] * m[9];

  result[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] +
              m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] +
              m[13] * m[3] * m[10];

  result[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] -
              m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] -
              m[12] * m[3] * m[10];

  result[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] +
              m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] +
              m[12] * m[3] * m[9];

  result[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] -
               m[8] * m[1] * m[14] + m[8] * m[2] * m[13] +
               m[12] * m[1] * m[10] - m[12] * m[2] * m[9];

  result[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
              m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];

  result[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
              m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];

  result[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
               m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];

  result[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] +
               m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] +
               m[12] * m[2] * m[5];

  result[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
              m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];

  result[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
              m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];

  result[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
               m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];

  result[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
               m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

  float det = m[0] * result[0] + m[1] * result[4] + m[2] * result[8] +
              m[3] * result[12];
  det = 1.0 / det;

  for (uint32_t i = 0; i < 16; i++)
    resultOut[i] = result[i] * det;

  return resultOut;
}

//////////////////////////////////////////////////
LuxCoreEngineMesh::LuxCoreEngineMesh()
{
  this->subMeshes = LuxCoreEngineSubMeshStorePtr(new LuxCoreEngineSubMeshStore);

  memset(this->scaleMatrix, 0, 16 * sizeof(float));
  memset(this->rotationMatrix, 0, 16 * sizeof(float));
  memset(this->translationMatrix, 0, 16 * sizeof(float));
  memset(this->previousMatrix, 0, 16 * sizeof(float));

  this->scaleMatrix[0] = 1.0f;
  this->scaleMatrix[5] = 1.0f;
  this->scaleMatrix[10] = 1.0f;
  this->scaleMatrix[15] = 1.0f;

  this->rotationMatrix[0] = 1.0f;
  this->rotationMatrix[5] = 1.0f;
  this->rotationMatrix[10] = 1.0f;
  this->rotationMatrix[15] = 1.0f;

  this->translationMatrix[0] = 1.0f;
  this->translationMatrix[5] = 1.0f;
  this->translationMatrix[10] = 1.0f;
  this->translationMatrix[15] = 1.0f;

  this->previousMatrix[0] = 1.0f;
  this->previousMatrix[5] = 1.0f;
  this->previousMatrix[10] = 1.0f;
  this->previousMatrix[15] = 1.0f;
}

//////////////////////////////////////////////////
bool LuxCoreEngineMesh::HasParent() const
{
  // \todo(anyone) implement this function
  return false;
}

//////////////////////////////////////////////////
VisualPtr LuxCoreEngineMesh::Parent() const
{
  // \todo(anyone) implement this function
  return VisualPtr();
}

//////////////////////////////////////////////////
void LuxCoreEngineMesh::SetMaterial(MaterialPtr _material, bool _unique)
{
  if (SubMeshes()->Size() > 0)
  {
    for (unsigned int i = 0; i < SubMeshes()->Size(); i++)
    {
       scene->SceneLux()->Parse(
           luxrays::Property("scene.objects." +
               SubMeshes()->GetByIndex(i)->Name() + ".shape")
               (SubMeshes()->GetByIndex(i)->Name() + "-submesh")
        << luxrays::Property("scene.objects." +
               SubMeshes()->GetByIndex(i)->Name() + ".material")
               (SubMeshes()->GetByIndex(i)->Name() + "-texmap"));
    }
  }
  else
  {
    scene->SceneLux()->Parse(
        luxrays::Property("scene.objects." + Name() + ".shape")
            (Name() + "-mesh")
     << luxrays::Property("scene.objects." + Name() + ".material")
            (_material->Name()));
  }
}

//////////////////////////////////////////////////
void LuxCoreEngineMesh::UpdateTransformation()
{
  if (SubMeshes()->Size() > 0)
  {
    for (unsigned int i = 0; i < SubMeshes()->Size(); i++)
    {
      std::vector<float> result =
          multiplyMatrix(this->rotationMatrix, this->scaleMatrix);
      result = multiplyMatrix(this->translationMatrix, result.data());
      std::vector<float> inversePrevious = inverseMatrix(this->previousMatrix);
      std::vector<float> offsetMatrix =
          multiplyMatrix(inversePrevious.data(), result.data());
      scene->SceneLux()->UpdateObjectTransformation(
          SubMeshes()->GetByIndex(i)->Name(), offsetMatrix.data());
      memcpy(this->previousMatrix, result.data(), sizeof(float) * 16);
    }
  }
  else
  {
    std::vector<float> result =
        multiplyMatrix(this->rotationMatrix, this->scaleMatrix);
    result = multiplyMatrix(this->translationMatrix, result.data());
    std::vector<float> inversePrevious = inverseMatrix(this->previousMatrix);
    std::vector<float> offsetMatrix =
        multiplyMatrix(inversePrevious.data(), result.data());
    scene->SceneLux()->UpdateObjectTransformation(Name(), offsetMatrix.data());
    memcpy(this->previousMatrix, result.data(), sizeof(float) * 16);
  }
}

//////////////////////////////////////////////////
void LuxCoreEngineMesh::SetLocalPosition(double _x, double _y, double _z)
{
  this->translationMatrix[12] = _x;
  this->translationMatrix[13] = _y;
  this->translationMatrix[14] = _z;

  this->UpdateTransformation();
}

//////////////////////////////////////////////////
void LuxCoreEngineMesh::SetLocalScale(double _x, double _y, double _z)
{
  this->scaleMatrix[0] = _x;
  this->scaleMatrix[5] = _y;
  this->scaleMatrix[10] = _z;

  this->UpdateTransformation();
}

//////////////////////////////////////////////////
void LuxCoreEngineMesh::SetLocalRotation(double _r, double _p, double _y)
{
  this->rotationMatrix[0] = cos(_y) * cos(_p);
  this->rotationMatrix[1] = sin(_y) * cos(_p);
  this->rotationMatrix[2] = -sin(_p);
  this->rotationMatrix[4] = (cos(_y) * sin(_p) * sin(_r)) - (sin(_y) * cos(_r));
  this->rotationMatrix[5] = (sin(_y) * sin(_p) * sin(_r)) + (cos(_y) * cos(_r));
  this->rotationMatrix[6] = cos(_p) * sin(_r);
  this->rotationMatrix[8] = (cos(_y) * sin(_p) * cos(_r)) + (sin(_y) * sin(_r));
  this->rotationMatrix[9] = (sin(_y) * sin(_p) * cos(_r)) - (cos(_y) * sin(_r));
  this->rotationMatrix[10] = cos(_p) * cos(_r);

  this->UpdateTransformation();
}

//////////////////////////////////////////////////
void LuxCoreEngineMesh::SetName(std::string name)
{
  this->name = name;
}

//////////////////////////////////////////////////
void LuxCoreEngineMesh::AddSubMesh(const LuxCoreEngineSubMeshPtr _child)
{
  this->subMeshes->Add(_child);
}

//////////////////////////////////////////////////
SubMeshStorePtr LuxCoreEngineMesh::SubMeshes() const
{
  return this->subMeshes;
}

//////////////////////////////////////////////////
void LuxCoreEngineSubMesh::SetMaterialImpl(MaterialPtr /*_material*/)
{
  // \todo(anyone) Implement this function
}

//////////////////////////////////////////////////
void LuxCoreEngineSubMesh::SetName(std::string name)
{
  this->name = name;
}
