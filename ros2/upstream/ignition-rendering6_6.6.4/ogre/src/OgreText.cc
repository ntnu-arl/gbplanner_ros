/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that windows types like LONGLONG are defined
  #include <windows.h>
#endif

#include <gz/common/Util.hh>
#include <gz/common/Console.hh>

#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreText.hh"

#define POS_TEX_BINDING    0
#define COLOUR_BINDING     1

class gz::rendering::OgreMovableText
  : public Ogre::MovableObject, public Ogre::Renderable
{
  /// \brief Constructor
  public: OgreMovableText();

  /// \brief Destructor
  public: virtual ~OgreMovableText();

  /// \brief Set the font. Valid fonts are defined in
  /// media/fonts/ignition-rendering.fontdef
  /// \param[in] _font Name of the font
  public: void SetFontName(const std::string &_font);

  /// \brief Set the text to display.
  /// \param[in] _text The text to display.
  public: void SetTextString(const std::string &_text);

  /// \brief Set the text color.
  /// \param[in] _color Text color.
  public: void SetColor(const gz::math::Color &_color);

  /// \brief Set the height of the character in meters.
  /// \param[in] _height Height of the characters.
  public: void SetCharHeight(const float _height);

  /// \brief Set the width of spaces between words.
  /// \param[in] _width Space width
  public: void SetSpaceWidth(const float _width);

  /// \brief Set the alignment of the text
  /// \param[in] _horizAlign Horizontal alignment
  /// \param[in] _vertAlign Vertical alignment
  public: void SetTextAlignment(const TextHorizontalAlign &_horizAlign,
                                const TextVerticalAlign &_vertAlign);

  /// \brief Set the baseline height of the text
  /// \param[in] _height Baseline height
  public: void SetBaseline(const float _baseline);

  /// \brief True = text always is displayed ontop.
  /// \param[in] _show Set to true to render the text on top of
  /// all other drawables.
  public: void SetShowOnTop(const bool _onTop);

  /// \brief Get the axis aligned bounding box of the text.
  /// \return The axis aligned bounding box.
  public: gz::math::AxisAlignedBox AABB() const;

  /// \brief Setup the geometry based on input text string.
  public: void SetupGeometry();

  /// \brief Update color of text.
  public: void UpdateColors();

  /// \brief Update material properties, mainly if onTop has changed.
  public: void UpdateMaterial();

  /// \brief Update font.
  public: void UpdateFont();

  /// \brief Update all text properties if dirty.
  public: void Update();

  /// \brief Set font name implementation.
  /// \param-in] _font Name of font
  public: void SetFontNameImpl(const std::string &_font);

  /// \internal
  /// \brief Method to allow a caller to abstractly iterate over the
  /// renderable instances.
  /// \param[in] _visitor Renderable instances to visit
  /// \param[in] _debug True if set to debug
  public: virtual void visitRenderables(Ogre::Renderable::Visitor *_visitor,
      bool _debug = false) override;

  /// \internal
  /// \brief Get the world transform (from MovableObject)
  protected: void getWorldTransforms(Ogre::Matrix4 *_xform) const override;

  /// \internal
  /// \brief Get the bounding radiu (from MovableObject)
  protected: float getBoundingRadius() const override;

  /// \internal
  /// \brief Get the squared view depth (from MovableObject)
  protected: float getSquaredViewDepth(const Ogre::Camera *_cam) const
      override;

  /// \internal
  /// \brief Get the render operation
  protected: void getRenderOperation(Ogre::RenderOperation &_op) override;

  /// \internal
  /// \brief Get the material
  protected: const Ogre::MaterialPtr &getMaterial() const override;

  /// \internal
  /// \brief Get the lights
  /// \deprecated Function has never returned meaningful values
  protected: const Ogre::LightList &getLights() const override;

  /// \internal
  private: const Ogre::AxisAlignedBox &getBoundingBox() const override;

  /// \internal
  private: const Ogre::String &getMovableType() const override;

  /// \internal
  private: void _notifyCurrentCamera(Ogre::Camera *_cam) override;

  /// \internal
  private: void _updateRenderQueue(Ogre::RenderQueue *_queue) override;

  /// \brief Flag to indicate text properties have changed
  private: bool textDirty = false;

  /// \brief Flag to indicate text color has changed
  private: bool colorDirty = true;

  /// \brief Flag to indicate text font has changed
  private: bool fontDirty = true;

  /// \brief Flag to indicate text material has changed
  private: bool materialDirty = false;

  /// \brief Bounding radius
  private: float radius = 0;

  /// \brief Viewport aspect coefficient
  private: float viewportAspectCoef = 0.75;

  /// \brief Ogre render operation
  private: Ogre::RenderOperation renderOp;

  /// \brief Axis aligned box
  private: Ogre::AxisAlignedBox *aabb = nullptr;

  /// \brief Pointer to camera which the text is facing - never set.
  private: Ogre::Camera *camera = nullptr;

  /// \brief Pointer to font
  private: Ogre::Font *font = nullptr;

  /// \brief Text ogreMaterial
  private: Ogre::MaterialPtr ogreMaterial;

  /// \brief Keep an empty list of lights.
  private: Ogre::LightList lightList;

  /// \brief Font name, such as "Liberation Sans"
  private: std::string fontName;

  /// \brief Text being displayed
  private: std::string text;

  /// \brief Text color
  private: gz::math::Color color;

  /// \brief Character height in meters
  private: float charHeight = 0.0;

  /// \brief Width of space between letters
  private: float spaceWidth = 0.0;

  /// \brief Horizontal alignment
  private: TextHorizontalAlign horizontalAlign;

  /// \brief Vertical alignment
  private: TextVerticalAlign verticalAlign;

  /// \brief Baseline height in meters.
  private: float baseline = 0.0;

  /// \brief True for text to be displayed on top of other objects in the
  /// scene.
  private: bool onTop = false;
};

/// \brief Private data for the OgreText class.
class gz::rendering::OgreTextPrivate
{
  /// \brief Text materal
  public: OgreMaterialPtr material;

  /// Pointer to ogre movable text object
  public: std::unique_ptr<OgreMovableText> ogreObj;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreMovableText::OgreMovableText()
{
  this->renderOp.vertexData = nullptr;
  this->aabb = new Ogre::AxisAlignedBox;
}

//////////////////////////////////////////////////
OgreMovableText::~OgreMovableText()
{
  delete this->renderOp.vertexData;
  delete this->aabb;
}

//////////////////////////////////////////////////
void OgreMovableText::SetFontName(const std::string &_font)
{
  if (this->fontName != _font)
  {
    this->fontName = _font;
    this->fontDirty = true;
  }
}

//////////////////////////////////////////////////
void OgreMovableText::SetTextString(const std::string &_text)
{
  if (this->text != _text)
  {
    this->text = _text;
    this->textDirty = true;
  }
}

//////////////////////////////////////////////////
void OgreMovableText::SetColor(const math::Color &_color)
{
  if (this->color != _color)
  {
    this->color = _color;
    this->colorDirty = true;
  }
}

//////////////////////////////////////////////////
void OgreMovableText::SetCharHeight(const float _height)
{
  if (!math::equal(this->charHeight, _height))
  {
    this->charHeight = _height;
    this->textDirty = true;
  }
}

//////////////////////////////////////////////////
void OgreMovableText::SetSpaceWidth(const float _width)
{
  if (!math::equal(this->spaceWidth, _width))
  {
    this->spaceWidth = _width;
    this->textDirty = true;
  }
}

//////////////////////////////////////////////////
void OgreMovableText::SetTextAlignment(const TextHorizontalAlign &_horizAlign,
                                       const TextVerticalAlign &_vertAlign)
{
  if (this->horizontalAlign != _horizAlign)
  {
    this->horizontalAlign = _horizAlign;
    this->textDirty = true;
  }
  if (this->verticalAlign != _vertAlign)
  {
    this->verticalAlign = _vertAlign;
    this->textDirty = true;
  }
}

//////////////////////////////////////////////////
void OgreMovableText::SetBaseline(const float _baseline)
{
  if (math::equal(this->baseline, _baseline))
  {
    this->baseline = _baseline;
    this->textDirty = true;
  }
}

//////////////////////////////////////////////////
void OgreMovableText::SetShowOnTop(const bool _onTop)
{
  if (this->onTop != _onTop)
  {
    this->onTop = _onTop;
    this->materialDirty = true;
  }
}

//////////////////////////////////////////////////
math::AxisAlignedBox OgreMovableText::AABB() const
{
  return math::AxisAlignedBox(
      math::Vector3d(this->aabb->getMinimum().x,
                    this->aabb->getMinimum().y,
                    this->aabb->getMinimum().z),
      math::Vector3d(this->aabb->getMaximum().x,
                    this->aabb->getMaximum().y,
                    this->aabb->getMaximum().z));
}

//////////////////////////////////////////////////
void OgreMovableText::UpdateFont()
{
  this->SetFontNameImpl(this->fontName);
}

//////////////////////////////////////////////////
void OgreMovableText::SetFontNameImpl(const std::string &_newFontName)
{
  if ((Ogre::MaterialManager::getSingletonPtr()->resourceExists(
          this->mName + "Material")))
  {
    Ogre::MaterialManager::getSingleton().remove(this->mName + "Material");
  }

  if (this->fontName != _newFontName ||
#if OGRE_VERSION_LT_1_11_0
      this->ogreMaterial.isNull() || !this->font)
#else
      this->ogreMaterial == nullptr || !this->font)
#endif
  {
    auto ogreFont = (Ogre::Font*)Ogre::FontManager::getSingleton()
        .getByName(_newFontName,
        Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME).get();

    if (!ogreFont)
    {
      ignerr << "Could not find font " + _newFontName << std::endl;
    }
    this->font = ogreFont;
    this->fontName = _newFontName;

    this->font->load();

#if OGRE_VERSION_LT_1_11_0
    if (!this->ogreMaterial.isNull())
#else
    if (this->ogreMaterial)
#endif
    {
      Ogre::MaterialManager::getSingletonPtr()->remove(
          this->ogreMaterial->getName());
#if OGRE_VERSION_LT_1_11_0
      this->ogreMaterial.setNull();
#else
      this->ogreMaterial = nullptr;
#endif
    }

    this->ogreMaterial = this->font->getMaterial()->clone(
        this->mName + "Material");

    if (!this->ogreMaterial->isLoaded())
      this->ogreMaterial->load();

    this->ogreMaterial->setDepthCheckEnabled(!this->onTop);
    this->ogreMaterial->setDepthBias(!this->onTop, 0);
    this->ogreMaterial->setDepthWriteEnabled(this->onTop);
    this->ogreMaterial->setLightingEnabled(false);

    this->textDirty = true;
  }
}

//////////////////////////////////////////////////
void OgreMovableText::SetupGeometry()
{
  IGN_ASSERT(this->font, "font class member is null");
#if OGRE_VERSION_LT_1_11_0
  IGN_ASSERT(!this->ogreMaterial.isNull(), "ogreMaterial class member is null");
#else
  IGN_ASSERT(this->ogreMaterial, "ogreMaterial class member is null");
#endif

  Ogre::VertexDeclaration *decl = nullptr;
  Ogre::VertexBufferBinding *bind = nullptr;
  Ogre::HardwareVertexBufferSharedPtr ptbuf;
  Ogre::HardwareVertexBufferSharedPtr cbuf;
  float *pVert = nullptr;
  float largestWidth = 0;
  float left = 0;
  float top = 0;
  size_t offset = 0;
  float maxSquaredRadius = 0.0f;
  bool first = true;
  std::string::iterator i;
  bool newLine = true;
  float len = 0.0f;

  // for calculation of AABB
  Ogre::Vector3 min(0, 0, 0);
  Ogre::Vector3 max(0, 0, 0);
  Ogre::Vector3 currPos(0, 0, 0);

  auto vertexCount = static_cast<unsigned int>(this->text.size() * 6);

  if (this->renderOp.vertexData)
  {
    // Removed this test as it causes problems when replacing a caption
    // of the same size: replacing "Hello" with "hello"
    // as well as when changing the text alignment
    // if (this->renderOp.vertexData->vertexCount != vertexCount)
    {
      delete this->renderOp.vertexData;
      this->renderOp.vertexData = nullptr;
      this->colorDirty = true;
    }
  }

  if (!this->renderOp.vertexData)
    this->renderOp.vertexData = new Ogre::VertexData();

  this->renderOp.indexData = 0;
  this->renderOp.vertexData->vertexStart = 0;
  this->renderOp.vertexData->vertexCount = vertexCount;
  this->renderOp.operationType =
      Ogre::RenderOperation::OT_TRIANGLE_LIST;
  this->renderOp.useIndexes = false;

  decl = this->renderOp.vertexData->vertexDeclaration;
  bind = this->renderOp.vertexData->vertexBufferBinding;

  // create/bind positions/tex.ccord. buffer
  if (!decl->findElementBySemantic(Ogre::VES_POSITION))
    decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT3,
                     Ogre::VES_POSITION);

  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (!decl->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES))
    decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT2,
                     Ogre::VES_TEXTURE_COORDINATES, 0);

  ptbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            decl->getVertexSize(POS_TEX_BINDING),
            this->renderOp.vertexData->vertexCount,
            Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

  bind->setBinding(POS_TEX_BINDING, ptbuf);

  // Colours - store these in a separate buffer because they change less often
  if (!decl->findElementBySemantic(Ogre::VES_DIFFUSE))
    decl->addElement(COLOUR_BINDING, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

  cbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
           decl->getVertexSize(COLOUR_BINDING),
           this->renderOp.vertexData->vertexCount,
           Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

  bind->setBinding(COLOUR_BINDING, cbuf);

  pVert = static_cast<float*>(ptbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // Derive space width from a capital A
  if (math::equal(this->spaceWidth, 0.0f))
  {
    this->spaceWidth = this->font->getGlyphAspectRatio('A') *
        this->charHeight * 2.0;
  }

  if (this->verticalAlign == TextVerticalAlign::TOP)
  {
    // Raise the first line of the caption
    top += this->charHeight;
    for (i = this->text.begin(); i != this->text.end(); ++i)
    {
      if (*i == '\n')
        top += this->charHeight * 2.0;
    }
  }

  for (i = this->text.begin(); i != this->text.end(); ++i)
  {
    if (newLine)
    {
      len = 0.0;
      for (std::string::iterator j = i; j != this->text.end(); ++j)
      {
        Ogre::Font::CodePoint character = *j;
        if (character == '\n')
        {
          break;
        }
        // space
        else if (character == ' ')
        {
          len += this->spaceWidth;
        }
        else
        {
          len += this->font->getGlyphAspectRatio(character) *
                 this->charHeight * 2.0 *
                 this->viewportAspectCoef;
        }
      }

      newLine = false;
    }

    Ogre::Font::CodePoint character = (*i);
    if (character == '\n')
    {
      left = 0.0;
      top -= this->charHeight * 2.0;
      newLine = true;

      // Also reduce tri count
      this->renderOp.vertexData->vertexCount -= 6;
      continue;
    }
    // space
    else if (character == ' ')
    {
      // Just leave a gap, no tris
      left += this->spaceWidth;

      // Also reduce tri count
      this->renderOp.vertexData->vertexCount -= 6;
      continue;
    }

    float horizHeight = this->font->getGlyphAspectRatio(character) *
                         this->viewportAspectCoef;

    auto &uvRect = this->font->getGlyphTexCoords(character);

    // each vert is (x, y, z, u, v)
    //------------------------------------------------------------------------
    // First tri
    //
    // Upper left
    if (this->horizontalAlign == TextHorizontalAlign::LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);

    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.top;

    // Deal with bounds
    if (this->horizontalAlign == TextHorizontalAlign::LEFT)
      currPos = Ogre::Vector3(left, top, 0);
    else
      currPos = Ogre::Vector3(left - (len/2.0), top, 0);

    if (first)
    {
      min = max = currPos;
      maxSquaredRadius = currPos.squaredLength();
      first = false;
    }
    else
    {
      min.makeFloor(currPos);
      max.makeCeil(currPos);
      maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());
    }

    top -= this->charHeight * 2.0;

    // Bottom left
    if (this->horizontalAlign == TextHorizontalAlign::LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len / 2.0);

    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.bottom;

    // Deal with bounds
    if (this->horizontalAlign == TextHorizontalAlign::LEFT)
      currPos = Ogre::Vector3(left, top, 0);
    else
      currPos = Ogre::Vector3(left - (len/2), top, 0);

    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());


    top += this->charHeight * 2.0;
    left += horizHeight * this->charHeight * 2.0;

    // Top right
    if (this->horizontalAlign == TextHorizontalAlign::LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);

    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.top;

    // Deal with bounds
    if (this->horizontalAlign == TextHorizontalAlign::LEFT)
      currPos = Ogre::Vector3(left, top, 0);
    else
      currPos = Ogre::Vector3(left - (len/2), top, 0);

    min.makeFloor(currPos);
    max.makeFloor(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());


    //------------------------------------------------------------------------

    //------------------------------------------------------------------------
    // Second tri
    //
    // Top right (again)
    if (this->horizontalAlign == TextHorizontalAlign::LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);
    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.top;

    // Deal with bounds
    currPos = Ogre::Vector3(left, top, 0);
    min.makeFloor(currPos);
    max.makeFloor(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());


    top -= this->charHeight * 2.0;
    left -= horizHeight  * this->charHeight * 2.0;

    // Bottom left (again)
    if (this->horizontalAlign == TextHorizontalAlign::LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);
    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.bottom;

    // Deal with bounds
    currPos = Ogre::Vector3(left, top, 0);
    min.makeFloor(currPos);
    max.makeFloor(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());


    left += horizHeight  * this->charHeight * 2.0;

    // Bottom right
    if (this->horizontalAlign == TextHorizontalAlign::LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);
    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.bottom;

    // Deal with bounds
    currPos = Ogre::Vector3(left, top, 0);
    min.makeFloor(currPos);
    max.makeFloor(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());

    //-------------------------------------------------------------------------


    // Go back up with top
    top += this->charHeight * 2.0;

    float currentWidth = (left + 1.0)/2.0;
    if (currentWidth > largestWidth)
    {
      largestWidth = currentWidth;
    }
  }

  // Unlock vertex buffer
  ptbuf->unlock();

  // update AABB/Sphere radius
  this->aabb->setMinimum(min);
  this->aabb->setMaximum(max);
  this->radius = Ogre::Math::Sqrt(maxSquaredRadius);

  if (this->colorDirty)
    this->UpdateColors();

  this->textDirty = false;
}

//////////////////////////////////////////////////
void OgreMovableText::UpdateColors()
{
  Ogre::RGBA clr;
  Ogre::HardwareVertexBufferSharedPtr vbuf;
  Ogre::RGBA *pDest{nullptr};
  unsigned int i;

  IGN_ASSERT(this->font, "font class member is null");
#if OGRE_VERSION_LT_1_11_0
  IGN_ASSERT(!this->ogreMaterial.isNull(), "ogreMaterial class member is null");
#else
  IGN_ASSERT(this->ogreMaterial, "ogreMaterial class member is null");
#endif

  // Convert to system-specific
  Ogre::ColourValue cv(this->color.R(), this->color.G(),
                       this->color.B(), this->color.A());
  Ogre::Root::getSingleton().convertColourValue(cv, &clr);

  vbuf = this->renderOp.vertexData->vertexBufferBinding->getBuffer(
         COLOUR_BINDING);

  pDest = static_cast<Ogre::RGBA*>(
      vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  for (i = 0; i < this->renderOp.vertexData->vertexCount; ++i)
  {
    *pDest++ = clr;
  }

  vbuf->unlock();
  this->colorDirty = false;
}

//////////////////////////////////////////////////
void OgreMovableText::UpdateMaterial()
{
#if OGRE_VERSION_LT_1_11_0
  if (this->ogreMaterial.isNull())
#else
  if (!this->ogreMaterial)
#endif
    return;

  this->ogreMaterial->setDepthBias(!this->onTop, 0);
  this->ogreMaterial->setDepthCheckEnabled(!this->onTop);
  this->ogreMaterial->setDepthWriteEnabled(this->onTop);

  this->materialDirty = false;
}

//////////////////////////////////////////////////
void OgreMovableText::Update()
{
  if (this->fontDirty)
    this->UpdateFont();
  if (this->textDirty)
    this->SetupGeometry();
  if (this->colorDirty)
    this->UpdateColors();
  if (this->materialDirty)
    this->UpdateMaterial();
}

//////////////////////////////////////////////////
const Ogre::AxisAlignedBox &OgreMovableText::getBoundingBox(void) const
{
  return *this->aabb;
}

//////////////////////////////////////////////////
const Ogre::String &OgreMovableText::getMovableType() const
{
  static Ogre::String movType = "OgreMovableText";
  return movType;
}

//////////////////////////////////////////////////
void OgreMovableText::getWorldTransforms(Ogre::Matrix4 *_xform) const
{
  if (this->isVisible() && this->camera)
  {
    Ogre::Matrix3 rot3x3, scale3x3 = Ogre::Matrix3::IDENTITY;

    // store rotation in a matrix
    this->camera->getDerivedOrientation().ToRotationMatrix(rot3x3);

    // parent node position
    Ogre::Vector3 ppos = mParentNode->_getDerivedPosition() +
                         Ogre::Vector3::UNIT_Z * this->baseline;

    // apply scale
    scale3x3[0][0] = mParentNode->_getDerivedScale().x / 2;
    scale3x3[1][1] = mParentNode->_getDerivedScale().y / 2;
    scale3x3[2][2] = mParentNode->_getDerivedScale().z / 2;

    // apply all transforms to xform
    *_xform = (rot3x3 * scale3x3);
    _xform->setTrans(ppos);
  }
}

//////////////////////////////////////////////////
float OgreMovableText::getBoundingRadius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
float OgreMovableText::getSquaredViewDepth(const Ogre::Camera * /*_cam*/) const
{
  return 0;
}

//////////////////////////////////////////////////
void OgreMovableText::getRenderOperation(Ogre::RenderOperation &_op)
{
  if (this->isVisible())
  {
    this->Update();
    _op = this->renderOp;
  }
}

//////////////////////////////////////////////////
const Ogre::MaterialPtr &OgreMovableText::getMaterial(void) const
{
#if OGRE_VERSION_LT_1_11_0
  IGN_ASSERT(!this->ogreMaterial.isNull(), "ogreMaterial class member is null");
#else
  IGN_ASSERT(this->ogreMaterial, "ogreMaterial class member is null");
#endif
  return this->ogreMaterial;
}

//////////////////////////////////////////////////
const Ogre::LightList &OgreMovableText::getLights(void) const
{
  return this->lightList;
}

//////////////////////////////////////////////////
void OgreMovableText::_notifyCurrentCamera(Ogre::Camera *_cam)
{
  this->camera = _cam;
}

//////////////////////////////////////////////////
void OgreMovableText::_updateRenderQueue(Ogre::RenderQueue* _queue)
{
  if (this->isVisible())
  {
    this->Update();

    _queue->addRenderable(this, mRenderQueueID,
                          OGRE_RENDERABLE_DEFAULT_PRIORITY);
  }
}

//////////////////////////////////////////////////
void OgreMovableText::visitRenderables(Ogre::Renderable::Visitor * /*_visitor*/,
                                 bool /*_debug*/)
{
  return;
}

//////////////////////////////////////////////////
OgreText::OgreText()
    : dataPtr(new OgreTextPrivate)
{
  this->dataPtr->ogreObj.reset(new OgreMovableText());
}

//////////////////////////////////////////////////
OgreText::~OgreText()
{
}

//////////////////////////////////////////////////
void OgreText::Init()
{
  // set default properties inherited from base
  this->SetFontName(this->fontName);
  this->SetCharHeight(this->charHeight);
  this->SetSpaceWidth(this->spaceWidth);
  this->SetBaseline(this->baseline);
  this->SetShowOnTop(this->onTop);
  this->SetColor(this->color);
  this->SetTextAlignment(this->horizontalAlign, this->verticalAlign);
}

//////////////////////////////////////////////////
void OgreText::PreRender()
{
  this->dataPtr->ogreObj->Update();
}

//////////////////////////////////////////////////
Ogre::MovableObject *OgreText::OgreObject() const
{
  return this->dataPtr->ogreObj.get();
}

//////////////////////////////////////////////////
void OgreText::SetMaterial(MaterialPtr _material, bool _unique)
{
  _material = (_unique) ? _material->Clone() : _material;

  OgreMaterialPtr derived =
      std::dynamic_pointer_cast<OgreMaterial>(_material);

  if (!derived)
  {
    ignerr << "Cannot assign ogreMaterial created by another render-engine"
        << std::endl;

    return;
  }

  this->SetMaterialImpl(derived);
}

//////////////////////////////////////////////////
void OgreText::SetMaterialImpl(OgreMaterialPtr _material)
{
  // only colors are support for now
  this->SetColor(_material->Diffuse());
  this->dataPtr->material = _material;
}

//////////////////////////////////////////////////
MaterialPtr OgreText::Material() const
{
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
void OgreText::SetFontName(const std::string &_font)
{
  BaseText::SetFontName(_font);
  this->dataPtr->ogreObj->SetFontName(_font);
}

//////////////////////////////////////////////////
void OgreText::SetTextString(const std::string &_text)
{
  BaseText::SetTextString(_text);
  this->dataPtr->ogreObj->SetTextString(_text);
}

//////////////////////////////////////////////////
void OgreText::SetColor(const math::Color &_color)
{
  BaseText::SetColor(_color);
  this->dataPtr->ogreObj->SetColor(_color);
}

//////////////////////////////////////////////////
void OgreText::SetCharHeight(const float _height)
{
  BaseText::SetCharHeight(_height);
  this->dataPtr->ogreObj->SetCharHeight(_height);
}

//////////////////////////////////////////////////
void OgreText::SetSpaceWidth(const float _width)
{
  BaseText::SetSpaceWidth(_width);
  this->dataPtr->ogreObj->SetSpaceWidth(_width);
}

//////////////////////////////////////////////////
void OgreText::SetTextAlignment(const TextHorizontalAlign &_horizAlign,
                                const TextVerticalAlign &_vertAlign)
{
  BaseText::SetTextAlignment(_horizAlign, _vertAlign);
  this->dataPtr->ogreObj->SetTextAlignment(_horizAlign, _vertAlign);
}

//////////////////////////////////////////////////
void OgreText::SetBaseline(const float _baseline)
{
  BaseText::SetBaseline(_baseline);
  this->dataPtr->ogreObj->SetBaseline(_baseline);
}

//////////////////////////////////////////////////
void OgreText::SetShowOnTop(const bool _onTop)
{
  BaseText::SetShowOnTop(_onTop);
  this->dataPtr->ogreObj->SetShowOnTop(_onTop);
}

//////////////////////////////////////////////////
math::AxisAlignedBox OgreText::AABB() const
{
  return this->dataPtr->ogreObj->AABB();
}
