/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_STORAGE_HH_
#define GZ_RENDERING_STORAGE_HH_

#include <memory>
#include <string>
#include "gz/rendering/config.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/Node.hh"
#include "gz/rendering/Light.hh"
#include "gz/rendering/Sensor.hh"
#include "gz/rendering/Visual.hh"
#include "gz/rendering/Geometry.hh"
#include "gz/rendering/Material.hh"
#include "gz/rendering/Mesh.hh"
#include "gz/rendering/Export.hh"

// An X11 header file defines a macro that conflicts with our function name,
// so undefine it here
#ifdef DestroyAll
#undef DestroyAll
#endif

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class Map Storage.hh ignition/rendering/Storage.hh
    /// \brief Storage map from std::string to template class T
    /// \tparam T The storage element type
    template <class T>
    class IGNITION_RENDERING_VISIBLE Map
    {
      /// \typedef TPtr
      /// \brief Shared pointer to T
      typedef std::shared_ptr<T> TPtr;

      /// \typedef const TPtr
      /// \brief Shared pointer to const T
      typedef std::shared_ptr<const T> ConstTPtr;

      /// \brief Deconstructor
      public: virtual ~Map() { }

      /// \brief Get the number of elements in this map
      /// \return The number of elements in this map
      public: virtual unsigned int Size() const = 0;

      /// \brief Determine if an elements is mapped to the given key
      /// \param[in] _key Key of the element in question
      /// \return True if an elements is mapped to the given key
      public: virtual bool ContainsKey(const std::string &_key) const = 0;

      /// \brief Determine if the given element exists in this map
      /// \param[in] _value Element in question
      /// \return True if the given element exists in this map
      public: virtual bool ContainsValue(ConstTPtr _value) const = 0;

      /// \brief Get the element mapped to the given key
      /// \param[in] _key Key of the element in question
      /// \return The element mapped to the given key
      public: virtual TPtr Get(const std::string &_key) const = 0;

      /// \brief Get the element at the given index
      /// \param[in] _index Index of the element in question
      /// \return The element at the given index
      public: virtual TPtr GetByIndex(unsigned int _index) const = 0;

      /// \brief Map to given element to the given key. If the given key is
      /// already in use, no work will be done.
      /// \param[in] _key Key of the new element
      /// \param[in] _value Element to be mapped
      /// \return True if successful
      public: virtual bool Put(const std::string &_key, TPtr _value) = 0;

      /// \brief Remove the element mapped to the given key. If the specified
      /// element does not exists, no work will be done.
      /// \param[in] _key Key of the element in question
      public: virtual void Remove(const std::string &_key) = 0;

      /// \brief Remove the given element. If the given element does not exist
      /// no work will be done.
      /// \param[in] _value Key of the element in question
      public: virtual void Remove(TPtr _value) = 0;

      /// \brief Remove all elements from this map
      public: virtual void RemoveAll() = 0;
    };

    /// \class Store Storage.hh ignition/rendering/Storage.hh
    /// \brief Multi-access storage structure of template class T. Template
    /// class T is expected to have functions GetId, GetName(), Destroy() which
    /// return unsigned int, std::string, and void respectively. This store
    /// will ensure that each element's name and ID are unique.
    /// \tparam T The storage element type
    template <class T>
    class IGNITION_RENDERING_VISIBLE Store
    {
      /// \typedef TPtr
      /// \brief Shared pointer to T
      typedef std::shared_ptr<T> TPtr;

      /// \typedef const TPtr
      /// \brief Shared pointer to const T
      typedef std::shared_ptr<const T> ConstTPtr;

      /// \brief Deconstructor
      public: virtual ~Store() { }

      /// \brief Get number of elements in this store
      /// \return The number of elements in this store
      public: virtual unsigned int Size() const = 0;

      /// \brief Determine if store contains the given element
      /// \param[in] _object The element in question
      /// \return True if this store contains the given element
      public: virtual bool Contains(ConstTPtr _object) const = 0;

      /// \brief Determine if store contains the element with the given ID
      /// \param[in] _id ID of the element in question
      /// \return True if this store contains the specified element
      public: virtual bool ContainsId(unsigned int _id) const = 0;

      /// \brief Determine if store contains the element with the given name
      /// \param[in] _name Name of the element in question
      /// \return True if this store contains the specified element
      public: virtual bool ContainsName(const std::string &_name) const = 0;

      /// \brief Get element with the given ID
      /// \param[in] _id ID of the desired element
      /// \return The specified element
      public: virtual TPtr GetById(unsigned int _id) const = 0;

      /// \brief Get element with the given name
      /// \param[in] _name Name of the desired element
      /// \return The specified element
      public: virtual TPtr GetByName(const std::string &_name) const = 0;

      /// \brief Get element at the given index
      /// \param[in] _index Index of the desired element
      /// \return The specified element
      public: virtual TPtr GetByIndex(unsigned int _index) const = 0;

      /// \brief Add given element. If the element has already been added
      /// or its name or ID conflict with other existing elements, then no
      /// work will be done.
      /// \param[in] _object Element to be added
      /// \return True if successful
      public: virtual bool Add(TPtr _object) = 0;

      /// \brief Remove given element. If the given element does not exists
      /// in this store, then no work will be done.
      /// \param[in] _object Element to be removed
      /// \return The removed element
      public: virtual TPtr Remove(TPtr _object) = 0;

      /// \brief Remove element with the given ID. If the specified element
      /// does not exists in this store, then no work will be done.
      /// \param[in] _id ID of the element to be removed
      /// \return The removed element
      public: virtual TPtr RemoveById(unsigned int _id) = 0;

      /// \brief Remove element with the given name. If the specified element
      /// does not exists in this store, then no work will be done.
      /// \param[in] _name Name of the element to be removed
      /// \return The removed element
      public: virtual TPtr RemoveByName(const std::string &_name) = 0;

      /// \brief Remove element at the given index. If the specified element
      /// does not exists in this store, then no work will be done.
      /// \param[in] _index Index of the element to be removed
      /// \return The removed element
      public: virtual TPtr RemoveByIndex(unsigned int _index) = 0;

      /// \brief Remove all elements from store
      public: virtual void RemoveAll() = 0;

      /// \brief Remove and destroy given element. If the given element does
      /// not exists, in this store, then no work will be done.
      /// \param[in] _object Element to be removed
      public: virtual void Destroy(TPtr _object) = 0;

      /// \brief Remove and destroy element with the given ID. If the specified
      /// element does not exists in this store, then no work will be done.
      /// \param[in] _id ID of the element to be removed
      public: virtual void DestroyById(unsigned int _id) = 0;

      /// \brief Remove and destroy element with the given name. If the
      /// specified element does not exists in this store, then no work
      /// will be done.
      /// \param[in] _name Name of the element to be removed
      public: virtual void DestroyByName(const std::string &_name) = 0;

      /// \brief Remove and destroy element at the given index. If the
      /// specified element does not exists in this store, then no work
      /// will be done.
      /// \param[in] _index Index of the element to be removed
      public: virtual void DestroyByIndex(unsigned int _index) = 0;

      /// \brief Remove and destroy all elements in store
      public: virtual void DestroyAll() = 0;
    };

    /// \class CompositeStore Storage.hh ignition/rendering/Storage.hh
    /// ignition/rendering/CompositeStore.hh
    /// \brief Represents a collection of Store objects, collectively working as
    /// a single composite store.
    /// \tparam T The storage element type
    template <class T>
    class IGNITION_RENDERING_VISIBLE CompositeStore :
      public Store<T>
    {
      /// \typedef TPtr
      /// \brief Shared pointer to T
      typedef std::shared_ptr<T> TPtr;

      /// \typedef TPtr
      /// \brief Store of T
      typedef Store<T> TStore;

      /// \typedef TStorePtr
      /// \brief Shared pointer to TStore
      typedef std::shared_ptr<TStore> TStorePtr;

      /// \typedef const TStorePtr
      /// \brief Shared pointer to const TStore
      typedef std::shared_ptr<const TStore> ConstTStorePtr;

      /// \brief Deconstructor
      public: virtual ~CompositeStore() { }

      /// \brief Get number of Stores
      /// \return The number of Stores
      public: virtual unsigned int GetStoreCount() const = 0;

      /// \brief Determine if given store exists
      /// \param[in] _store Store in question
      /// \return True if given store exists
      public: virtual bool ContainsStore(ConstTStorePtr _store) const = 0;

      /// \brief Add the given store. If the given store already exists, then
      /// no work will be done.
      /// \param[in] _store Store to be added
      /// \return True if successful
      public: virtual bool AddStore(TStorePtr _store) = 0;

      /// \brief Get store at the given index. If no such store exists NULL
      /// will be returned.
      /// \param[in] _index Index of the desired store
      /// \return The specified store
      public: virtual TStorePtr GetStore(unsigned int _index) const = 0;

      /// \brief Remove given store. If no such store exists no
      /// work will be done and NULL will be returned.
      /// \param[in] _store Store to be removed
      /// \return The removed store
      public: virtual TStorePtr RemoveStore(TStorePtr _store) = 0;

      /// \brief Remove store at the given index. If no such store exists no
      /// work will be done and NULL will be returned.
      /// \param[in] _index Index of the store to be removed
      /// \return The removed store
      public: virtual TStorePtr RemoveStore(unsigned int _index) = 0;
    };

    /// \class StoreWrapper Storage.hh ignition/rendering/StoreWrapper.hh
    /// \brief Simple wrapper class that allows a Store of type Derived to be
    /// treated as a Store of type Base, where Derived is some class derived
    /// from Base. This is useful in storing Stores of different derived types
    /// in a single CompositeStore instance.
    /// \tparam T New storage type
    /// \tparam U Some derived class of T
    template <class T, class U>
    class IGNITION_RENDERING_VISIBLE StoreWrapper :
      public Store<T>
    {
      public: virtual ~StoreWrapper() { }
    };

// armhf failed to build with this code. It can not be removed for the rest
// of arches to keep ABI but should be removed in major versions unreleased
// see https://github.com/ignitionrobotics/ign-rendering/pull/457
#ifndef __ARM_PCS_VFP
    template class Store<Scene>;
    template class Store<Node>;
    template class Store<Light>;
    template class Store<Sensor>;
    template class Store<Visual>;
    template class Store<Geometry>;
    template class Store<SubMesh>;
    template class Map<Material>;
    template class CompositeStore<Node>;
#endif

    /// \def SceneStore
    /// \brief Store of Scene
    typedef Store<Scene> SceneStore;

    /// \def NodeStore
    /// \brief Store of Node
    typedef Store<Node> NodeStore;

    /// \def LightStore
    /// \brief Store of Light
    typedef Store<Light> LightStore;

    /// \def SensorStore
    /// \brief Store of Sensor
    typedef Store<Sensor> SensorStore;

    /// \def VisualStore
    /// \brief Store of Visual
    typedef Store<Visual> VisualStore;

    /// \def GeometryStore
    /// \brief Store of Geometry
    typedef Store<Geometry> GeometryStore;

    /// \def SubMeshStore
    /// \brief Store of SubMesh
    typedef Store<SubMesh> SubMeshStore;

    /// \def MaterialMap
    /// \brief Material Map
    typedef Map<Material> MaterialMap;

    /// \def NodeCompositeStore
    /// \brief Node Composite Store
    typedef CompositeStore<Node> NodeCompositeStore;

    /// \def SceneStorePtr
    /// \brief Shared pointer to SceneStore
    typedef std::shared_ptr<SceneStore> SceneStorePtr;

    /// \def NodeStorePtr
    /// \brief Shared pointer to NodeStore
    typedef std::shared_ptr<NodeStore> NodeStorePtr;

    /// \def LightStorePtr
    /// \brief Shared pointer to LightStore
    typedef std::shared_ptr<LightStore> LightStorePtr;

    /// \def SensorStorePtr
    /// \brief Shared pointer to SensorStore
    typedef std::shared_ptr<SensorStore> SensorStorePtr;

    /// \def VisualStorePtr
    /// \brief Shared pointer to VisualStore
    typedef std::shared_ptr<VisualStore> VisualStorePtr;

    /// \def GeometryStorePtr
    /// \brief Shared pointer to GeometryStore
    typedef std::shared_ptr<GeometryStore> GeometryStorePtr;

    /// \def SubMeshStorePtr
    /// \brief Shared pointer to SubMeshStore
    typedef std::shared_ptr<SubMeshStore> SubMeshStorePtr;

    /// \def MaterialMapPtr
    /// \brief Shared pointer to MaterialMap
    typedef std::shared_ptr<MaterialMap> MaterialMapPtr;

    /// \def NodeCompositeStorePtr
    /// \brief Shared pointer to NodeCompositeStore
    typedef std::shared_ptr<NodeCompositeStore> NodeCompositeStorePtr;
    }
  }
}
#endif
