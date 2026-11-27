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
#ifndef GZ_RENDERING_BASE_BASESTORAGE_HH_
#define GZ_RENDERING_BASE_BASESTORAGE_HH_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/Storage.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    //////////////////////////////////////////////////
    template <class T, class U>
    class BaseMap :
      public Map<T>
    {
      typedef std::shared_ptr<T> TPtr;

      typedef std::shared_ptr<const T> ConstTPtr;

      typedef std::shared_ptr<U> UPtr;

      typedef std::map<std::string, UPtr> UMap;

      typedef typename UMap::iterator UIter;

      typedef typename UMap::const_iterator ConstUIter;

      public: BaseMap();

      public: virtual ~BaseMap();

      public: virtual unsigned int Size() const;

      public: virtual bool ContainsKey(const std::string &_key) const;

      public: virtual bool ContainsValue(ConstTPtr _value) const;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      public: virtual TPtr Get(const std::string &_key) const;

      public: virtual TPtr GetByIndex(unsigned int _index) const;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      public: virtual bool Put(const std::string &_key, TPtr _value);

      public: virtual void Remove(const std::string &_key);

      public: virtual void Remove(TPtr _value);

      public: virtual void RemoveAll();

      public: virtual UPtr Derived(const std::string &_key) const;

      public: virtual UPtr DerivedByIndex(unsigned int _index) const;

      protected: virtual bool IsValidIter(ConstUIter _iter) const;

      protected: UMap map;
    };

    //////////////////////////////////////////////////
    template <class T, class U>
    class BaseStore :
      public Store<T>
    {
      typedef std::shared_ptr<T> TPtr;

      typedef std::shared_ptr<const T> ConstTPtr;

      typedef std::shared_ptr<U> UPtr;

      typedef std::map<std::string, UPtr> UStore;

      typedef typename UStore::iterator UIter;

      typedef typename UStore::const_iterator ConstUIter;

      public: BaseStore();

      public: virtual ~BaseStore();

      public: virtual unsigned int Size() const;

      public: virtual bool Contains(ConstTPtr _object) const;

      public: virtual bool ContainsId(unsigned int _id) const;

      public: virtual bool ContainsName(const std::string &_name) const;

      public: virtual TPtr GetById(unsigned int _id) const;

      public: virtual TPtr GetByName(const std::string &_name) const;

      public: virtual TPtr GetByIndex(unsigned int _index) const;

      public: virtual bool Add(TPtr _object);

      public: virtual TPtr Remove(TPtr _object);

      public: virtual TPtr RemoveById(unsigned int _id);

      public: virtual TPtr RemoveByName(const std::string &_name);

      public: virtual TPtr RemoveByIndex(unsigned int _index);

      public: virtual void RemoveAll();

      public: virtual void Destroy(TPtr _object);

      public: virtual void DestroyById(unsigned int _id);

      public: virtual void DestroyByName(const std::string &_name);

      public: virtual void DestroyByIndex(unsigned int _index);

      public: virtual void DestroyAll();

      public: virtual UPtr DerivedById(unsigned int _id) const;

      public: virtual UPtr DerivedByName(const std::string &_name) const;

      public: virtual UPtr DerivedByIndex(unsigned int _index) const;

      public: virtual bool AddDerived(UPtr _object);

      public: virtual UPtr RemoveDerived(UPtr _object);

      public: virtual UPtr RemoveDerivedById(unsigned int _id);

      public: virtual UPtr RemoveDerivedByName(const std::string &_name);

      public: virtual UPtr RemoveDerivedByIndex(unsigned int _index);

      /// \brief Return an iterator to the beginning
      /// \returns Iterator to beginning
      public: virtual UIter Begin();

      /// \brief Return an iterator to the end
      /// \returns Iterator to end
      public: virtual UIter End();

      protected: virtual ConstUIter ConstIter(ConstTPtr _object) const;

      protected: virtual ConstUIter ConstIterById(unsigned int _id) const;

      protected: virtual ConstUIter ConstIterByName(
                     const std::string &_name) const;

      protected: virtual ConstUIter ConstIterByIndex(unsigned int _index) const;

      protected: virtual UIter Iter(ConstTPtr _object);

      protected: virtual UIter IterById(unsigned int _id);

      protected: virtual UIter IterByName(const std::string &_name);

      protected: virtual UIter IterByIndex(unsigned int _index);

      protected: virtual bool AddImpl(UPtr _object);

      protected: virtual UPtr RemoveImpl(UIter _iter);

      protected: virtual void DestroyImpl(UIter _iter);

      protected: virtual bool IsValidIter(ConstUIter _iter) const;

      protected: virtual UIter RemoveConstness(ConstUIter _iter);

      protected: UStore store;
    };

    //////////////////////////////////////////////////
    template <class T>
    class BaseCompositeStore :
      public CompositeStore<T>
    {
      typedef std::shared_ptr<T> TPtr;

      typedef std::shared_ptr<const T> ConstTPtr;

      typedef Store<T> TStore;

      typedef std::shared_ptr<TStore> TStorePtr;

      typedef std::shared_ptr<const TStore> ConstTStorePtr;

      typedef std::vector<TStorePtr> TStoreList;

      typedef typename TStoreList::iterator TStoreIter;

      public: BaseCompositeStore();

      public: virtual ~BaseCompositeStore();

      public: virtual unsigned int Size() const;

      public: virtual bool Contains(ConstTPtr _object) const;

      public: virtual bool ContainsId(unsigned int _id) const;

      public: virtual bool ContainsName(const std::string &_name) const;

      public: virtual TPtr GetById(unsigned int _id) const;

      public: virtual TPtr GetByName(const std::string &_name) const;

      public: virtual TPtr GetByIndex(unsigned int _index) const;

      public: virtual bool Add(TPtr _object);

      public: virtual TPtr Remove(TPtr _object);

      public: virtual TPtr RemoveById(unsigned int _id);

      public: virtual TPtr RemoveByName(const std::string &_name);

      public: virtual TPtr RemoveByIndex(unsigned int _index);

      public: virtual void RemoveAll();

      public: virtual void Destroy(TPtr _object);

      public: virtual void DestroyById(unsigned int _id);

      public: virtual void DestroyByName(const std::string &_name);

      public: virtual void DestroyByIndex(unsigned int _index);

      public: virtual void DestroyAll();

      public: virtual unsigned int GetStoreCount() const;

      public: virtual bool ContainsStore(ConstTStorePtr _store) const;

      public: virtual TStorePtr GetStore(unsigned int _index) const;

      public: virtual bool AddStore(TStorePtr _store);

      public: virtual TStorePtr RemoveStore(TStorePtr _store);

      public: virtual TStorePtr RemoveStore(unsigned int _index);

      public: virtual TStorePtr RemoveStoreImpl(TStoreIter _iter);

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      protected: TStoreList stores;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };

    //////////////////////////////////////////////////
    template <class T, class U>
    class BaseStoreWrapper :
      public StoreWrapper<T, U>
    {
      typedef std::shared_ptr<T> TPtr;

      typedef std::shared_ptr<const T> ConstTPtr;

      typedef std::shared_ptr<U> UPtr;

      typedef std::shared_ptr<const U> ConstUPtr;

      typedef Store<U> UStore;

      typedef std::shared_ptr<UStore> UStorePtr;

      public: explicit BaseStoreWrapper(UStorePtr _store);

      public: virtual ~BaseStoreWrapper();

      public: virtual unsigned int Size() const;

      public: virtual bool Contains(ConstTPtr _object) const;

      public: virtual bool ContainsId(unsigned int _id) const;

      public: virtual bool ContainsName(const std::string &_name) const;

      public: virtual TPtr GetById(unsigned int _id) const;

      public: virtual TPtr GetByName(const std::string &_name) const;

      public: virtual TPtr GetByIndex(unsigned int _index) const;

      public: virtual bool Add(TPtr _object);

      public: virtual TPtr Remove(TPtr _object);

      public: virtual TPtr RemoveById(unsigned int _id);

      public: virtual TPtr RemoveByName(const std::string &_name);

      public: virtual TPtr RemoveByIndex(unsigned int _index);

      public: virtual void RemoveAll();

      public: virtual void Destroy(TPtr _object);

      public: virtual void DestroyById(unsigned int _id);

      public: virtual void DestroyByName(const std::string &_name);

      public: virtual void DestroyByIndex(unsigned int _index);

      public: virtual void DestroyAll();

      protected: UStorePtr store;
    };

    //////////////////////////////////////////////////
    class  BaseNodeCompositeStore :
      public BaseCompositeStore<Node>
    {
    };

    template <class T>
    class  BaseSceneStore :
      public BaseStore<Scene, T>
    {
    };

    template <class T>
    class  BaseNodeStore :
      public BaseStore<Node, T>
    {
    };

    template <class T>
    class  BaseLightStore :
      public BaseStore<Light, T>
    {
    };

    template <class T>
    class  BaseSensorStore :
      public BaseStore<Sensor, T>
    {
    };

    template <class T>
    class  BaseVisualStore :
      public BaseStore<Visual, T>
    {
    };

    template <class T>
    class  BaseGeometryStore :
      public BaseStore<Geometry, T>
    {
    };

    template <class T>
    class  BaseSubMeshStore :
      public BaseStore<SubMesh, T>
    {
    };

    template <class T>
    class  BaseMaterialMap :
      public BaseMap<Material, T>
    {
    };

    //////////////////////////////////////////////////
    template <class T, class U>
    BaseMap<T, U>::BaseMap()
    {
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    BaseMap<T, U>::~BaseMap()
    {
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    unsigned int BaseMap<T, U>::Size() const
    {
      return this->map.size();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseMap<T, U>::ContainsKey(const std::string &_key) const
    {
      return this->map.count(_key) > 0;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseMap<T, U>::ContainsValue(ConstTPtr _value) const
    {
      for (auto pair : this->map)
      {
        if (pair.second == _value) return true;
      }

      return false;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseMap<T, U>::TPtr
    BaseMap<T, U>::Get(const std::string &_key) const
    {
      return this->Derived(_key);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseMap<T, U>::TPtr
    BaseMap<T, U>::GetByIndex(unsigned int _index) const
    {
      return this->DerivedByIndex(_index);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseMap<T, U>::Put(const std::string &_key, TPtr _value)
    {
      if (_key.empty())
      {
        ignerr << "Cannot store item with empty key" << std::endl;
        return false;
      }

      if (!_value)
      {
        ignerr << "Cannot store null pointer value" << std::endl;
        return false;
      }

      if (this->ContainsKey(_key))
      {
        ignerr << "Item already registered with key: " << _key << std::endl;
        return false;
      }

      UPtr derived = std::dynamic_pointer_cast<U>(_value);

      if (!derived)
      {
        ignerr << "Cannot add item created by another render-engine"
              << std::endl;

        return false;
      }

      this->map[_key] = derived;
      return true;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseMap<T, U>::Remove(const std::string &_key)
    {
      auto iter = this->map.find(_key);

      if (this->IsValidIter(iter))
      {
        this->map.erase(iter);
      }
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseMap<T, U>::Remove(TPtr _value)
    {
      auto iter = this->map.begin();

      while (this->IsValidIter(iter))
      {
        if (iter->second == _value)
        {
          this->map.erase(iter);
          continue;
        }

        ++iter;
      }
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseMap<T, U>::RemoveAll()
    {
      this->map.clear();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseMap<T, U>::UPtr
    BaseMap<T, U>::Derived(const std::string &_key) const
    {
      auto iter = this->map.find(_key);
      return (this->IsValidIter(iter)) ? iter->second : nullptr;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseMap<T, U>::UPtr
    BaseMap<T, U>::DerivedByIndex(unsigned int _index) const
    {
      if (_index >= this->Size())
      {
        ignerr << "Invalid index: " << _index << std::endl;
        return nullptr;
      }

      auto iter = this->map.begin();
      std::advance(iter, _index);
      return iter->second;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseMap<T, U>::IsValidIter(ConstUIter _iter) const
    {
      return _iter != this->map.end();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    BaseStore<T, U>::BaseStore()
    {
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    BaseStore<T, U>::~BaseStore()
    {
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    unsigned int BaseStore<T, U>::Size() const
    {
      return this->store.size();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UIter
    BaseStore<T, U>::Begin()
    {
      return this->store.begin();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UIter
    BaseStore<T, U>::End()
    {
      return this->store.end();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStore<T, U>::Contains(ConstTPtr _object) const
    {
      auto iter = this->ConstIter(_object);
      return this->IsValidIter(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStore<T, U>::ContainsId(unsigned int _id) const
    {
      auto iter = this->ConstIterById(_id);
      return this->IsValidIter(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStore<T, U>::ContainsName(const std::string &_name) const
    {
      auto iter = this->ConstIterByName(_name);
      return this->IsValidIter(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::TPtr
    BaseStore<T, U>::GetById(unsigned int _id) const
    {
      return this->DerivedById(_id);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::TPtr
    BaseStore<T, U>::GetByName(const std::string &_name) const
    {
      return this->DerivedByName(_name);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::TPtr
    BaseStore<T, U>::GetByIndex(unsigned int _index) const
    {
      return this->DerivedByIndex(_index);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStore<T, U>::Add(TPtr _object)
    {
      if (!_object)
      {
        ignerr << "Cannot add null pointer" << std::endl;
        return false;
      }

      UPtr derived = std::dynamic_pointer_cast<U>(_object);

      if (!derived)
      {
        ignerr << "Cannot add item created by another render-engine"
              << std::endl;

        return false;
      }

      return this->AddImpl(derived);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::TPtr
    BaseStore<T, U>::Remove(TPtr _object)
    {
      auto iter = this->Iter(_object);
      return this->RemoveImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::TPtr
    BaseStore<T, U>::RemoveById(unsigned int _id)
    {
      return this->RemoveDerivedById(_id);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::TPtr
    BaseStore<T, U>::RemoveByName(const std::string &_name)
    {
      return this->RemoveDerivedByName(_name);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::TPtr
    BaseStore<T, U>::RemoveByIndex(unsigned int _index)
    {
      return this->RemoveDerivedByIndex(_index);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStore<T, U>::RemoveAll()
    {
      this->store.clear();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStore<T, U>::Destroy(TPtr _object)
    {
      auto iter = this->Iter(_object);
      this->DestroyImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStore<T, U>::DestroyById(unsigned int _id)
    {
      auto iter = this->IterById(_id);
      this->DestroyImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStore<T, U>::DestroyByName(const std::string &_name)
    {
      auto iter = this->IterByName(_name);
      this->DestroyImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStore<T, U>::DestroyByIndex(unsigned int _index)
    {
      auto iter = this->IterByIndex(_index);
      this->DestroyImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStore<T, U>::DestroyAll()
    {
      unsigned int i = this->Size();

      while (i > 0)
      {
        this->DestroyByIndex(--i);
      }
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UPtr
    BaseStore<T, U>::DerivedById(unsigned int _id) const
    {
      auto iter = this->ConstIterById(_id);
      return (this->IsValidIter(iter)) ? iter->second : nullptr;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UPtr
    BaseStore<T, U>::DerivedByName(const std::string &_name) const
    {
      auto iter = this->ConstIterByName(_name);
      return (this->IsValidIter(iter)) ? iter->second : nullptr;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UPtr
    BaseStore<T, U>::DerivedByIndex(unsigned int _index) const
    {
      auto iter = this->ConstIterByIndex(_index);
      return (this->IsValidIter(iter)) ? iter->second : nullptr;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStore<T, U>::AddDerived(UPtr _object)
    {
      if (!_object)
      {
        ignerr << "Cannot add null pointer" << std::endl;
        return false;
      }

      return this->AddImpl(_object);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UPtr
    BaseStore<T, U>::RemoveDerived(UPtr _object)
    {
      auto iter = this->Iter(_object);
      return this->RemoveImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UPtr
    BaseStore<T, U>::RemoveDerivedById(unsigned int _id)
    {
      auto iter = this->IterById(_id);
      return this->RemoveImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UPtr
    BaseStore<T, U>::RemoveDerivedByName(const std::string &_name)
    {
      auto iter = this->IterByName(_name);
      return this->RemoveImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UPtr
    BaseStore<T, U>::RemoveDerivedByIndex(unsigned int _index)
    {
      auto iter = this->IterByIndex(_index);
      return this->RemoveImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::ConstUIter
    BaseStore<T, U>::ConstIter(ConstTPtr _object) const
    {
      auto begin = this->store.begin();
      auto end = this->store.end();

      for (auto iter = begin; iter != end; ++iter)
      {
        if (iter->second == _object)
        {
          return iter;
        }
      }

      return end;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::ConstUIter
    BaseStore<T, U>::ConstIterById(unsigned int _id) const
    {
      auto begin = this->store.begin();
      auto end = this->store.end();

      for (auto iter = begin; iter != end; ++iter)
      {
        if (iter->second->Id() == _id)
        {
          return iter;
        }
      }

      return end;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::ConstUIter
    BaseStore<T, U>::ConstIterByName(const std::string &_name) const
    {
      return this->store.find(_name);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::ConstUIter
    BaseStore<T, U>::ConstIterByIndex(unsigned int _index) const
    {
      if (_index >= this->Size())
      {
        ignerr << "Invalid index: " << _index << std::endl;
        return this->store.end();
      }

      auto iter = this->store.begin();
      std::advance(iter, _index);
      return iter;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UIter
    BaseStore<T, U>::Iter(ConstTPtr _object)
    {
      auto iter = this->ConstIter(_object);
      return this->RemoveConstness(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UIter
    BaseStore<T, U>::IterById(unsigned int _id)
    {
      auto iter = this->ConstIterById(_id);
      return this->RemoveConstness(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UIter
    BaseStore<T, U>::IterByName(const std::string &_name)
    {
      auto iter = this->ConstIterByName(_name);
      return this->RemoveConstness(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UIter
    BaseStore<T, U>::IterByIndex(unsigned int _index)
    {
      auto iter = this->ConstIterByIndex(_index);
      return this->RemoveConstness(iter);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStore<T, U>::AddImpl(UPtr _object)
    {
      unsigned int id = _object->Id();
      std::string name = _object->Name();

      if (this->ContainsId(id))
      {
        ignerr << "Another item already exists with id: " << id << std::endl;
        return false;
      }

      if (this->ContainsName(name))
      {
        ignerr << "Another item already exists with name: " << name
            << std::endl;
        return false;
      }

      this->store[name] = _object;
      return true;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UPtr
    BaseStore<T, U>::RemoveImpl(UIter _iter)
    {
      if (!this->IsValidIter(_iter))
      {
        return nullptr;
      }

      UPtr result = _iter->second;
      this->store.erase(_iter);
      return result;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStore<T, U>::DestroyImpl(UIter _iter)
    {
      UPtr result = this->RemoveImpl(_iter);
      if (result) result->Destroy();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStore<T, U>::IsValidIter(ConstUIter _iter) const
    {
      return _iter != this->store.end();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStore<T, U>::UIter
    BaseStore<T, U>::RemoveConstness(ConstUIter _iter)
    {
      return (this->IsValidIter(_iter)) ?
          this->store.erase(_iter, _iter) : this->store.end();
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseCompositeStore<T>::BaseCompositeStore()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseCompositeStore<T>::~BaseCompositeStore()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseCompositeStore<T>::Size() const
    {
      unsigned int size = 0;

      for (auto store : this->stores)
      {
        size += store->Size();
      }

      return size;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseCompositeStore<T>::Contains(ConstTPtr _object) const
    {
      for (auto store : this->stores)
      {
        if (store->Contains(_object)) return true;
      }

      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseCompositeStore<T>::ContainsId(unsigned int _id) const
    {
      for (auto store : this->stores)
      {
        if (store->ContainsId(_id)) return true;
      }

      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseCompositeStore<T>::ContainsName(const std::string &_name) const
    {
      for (auto store : this->stores)
      {
        if (store->ContainsName(_name)) return true;
      }

      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TPtr
    BaseCompositeStore<T>::GetById(unsigned int _id) const
    {
      for (auto store : this->stores)
      {
        TPtr object = store->GetById(_id);
        if (object) return object;
      }

      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TPtr
    BaseCompositeStore<T>::GetByName(const std::string &_name) const
    {
      for (auto store : this->stores)
      {
        TPtr object = store->GetByName(_name);
        if (object) return object;
      }

      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TPtr
    BaseCompositeStore<T>::GetByIndex(unsigned int _index) const
    {
      unsigned int origIndex = _index;

      for (auto store : this->stores)
      {
        unsigned int size = store->Size();
        if (_index < size)
          return store->GetByIndex(_index);
        _index -= size;
      }

      ignerr << "Invalid index: " << origIndex << std::endl;
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseCompositeStore<T>::Add(TPtr)
    {
      ignerr << "Adding to BaseCompositeStore not supported" << std::endl;
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TPtr
    BaseCompositeStore<T>::Remove(TPtr _object)
    {
      TPtr result = nullptr;

      for (auto store : this->stores)
      {
        TPtr temp = store->Remove(_object);
        if (!result) result = temp;
      }

      return result;
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TPtr
    BaseCompositeStore<T>::RemoveById(unsigned int _id)
    {
      TPtr result = nullptr;

      for (auto store : this->stores)
      {
        TPtr temp = store->RemoveById(_id);
        if (!result) result = temp;
      }

      return result;
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TPtr
    BaseCompositeStore<T>::RemoveByName(const std::string &_name)
    {
      TPtr result = nullptr;

      for (auto store : this->stores)
      {
        TPtr temp = store->RemoveByName(_name);
        if (!result) result = temp;
      }

      return result;
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TPtr
    BaseCompositeStore<T>::RemoveByIndex(unsigned int _index)
    {
      TPtr result = nullptr;

      for (auto store : this->stores)
      {
        TPtr temp = store->RemoveByIndex(_index);
        if (!result) result = temp;
      }

      return result;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseCompositeStore<T>::RemoveAll()
    {
      for (auto store : this->stores)
      {
        store->RemoveAll();
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseCompositeStore<T>::Destroy(TPtr _object)
    {
      for (auto store : this->stores)
      {
        if (store->Contains(_object))
        {
          store->Destroy(_object);
          return;
        }
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseCompositeStore<T>::DestroyById(unsigned int _id)
    {
      for (auto store : this->stores)
      {
        store->DestroyById(_id);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseCompositeStore<T>::DestroyByName(const std::string &_name)
    {
      for (auto store : this->stores)
      {
        store->DestroyByName(_name);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseCompositeStore<T>::DestroyByIndex(unsigned int _index)
    {
      for (auto store : this->stores)
      {
        store->DestroyByIndex(_index);
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseCompositeStore<T>::DestroyAll()
    {
      for (auto store : this->stores)
      {
        store->DestroyAll();
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseCompositeStore<T>::GetStoreCount() const
    {
      return this->stores.size();
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseCompositeStore<T>::ContainsStore(ConstTStorePtr _store) const
    {
      auto begin = this->stores.begin();
      auto end = this->stores.end();
      auto iter = std::find(begin, end, _store);
      return iter != end;
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TStorePtr
    BaseCompositeStore<T>::GetStore(unsigned int _index) const
    {
      if (_index >= this->GetStoreCount())
      {
        ignerr << "Invalid store index: " << _index << std::endl;
        return nullptr;
      }

      return this->stores[_index];
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseCompositeStore<T>::AddStore(TStorePtr _store)
    {
      if (!_store)
      {
        ignerr << "Cannot all null store pointer" << std::endl;
        return false;
      }

      if (this->ContainsStore(_store))
      {
        ignerr << "Store has already been added" << std::endl;
        return false;
      }

      this->stores.push_back(_store);
      return true;
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TStorePtr
    BaseCompositeStore<T>::RemoveStore(TStorePtr _store)
    {
      auto begin = this->stores.begin();
      auto end = this->stores.end();
      auto iter = std::find(begin, end, _store);
      return this->RemoveStoreImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TStorePtr
    BaseCompositeStore<T>::RemoveStore(unsigned int _index)
    {
      if (_index >= this->GetStoreCount())
      {
        ignerr << "Invalid store index: " << _index << std::endl;
        return nullptr;
      }

      auto iter = this->stores.begin();
      std::advance(iter, _index);
      return this->RemoveStoreImpl(iter);
    }

    //////////////////////////////////////////////////
    template <class T>
    typename BaseCompositeStore<T>::TStorePtr
    BaseCompositeStore<T>::RemoveStoreImpl(TStoreIter _iter)
    {
      if (_iter == this->stores.end())
      {
        return nullptr;
      }

      TStorePtr result = *_iter;
      this->stores.erase(_iter);
      return result;
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    BaseStoreWrapper<T, U>::BaseStoreWrapper(UStorePtr _store) :
      store(_store)
    {
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    BaseStoreWrapper<T, U>::~BaseStoreWrapper()
    {
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    unsigned int BaseStoreWrapper<T, U>::Size() const
    {
      return this->store->Size();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStoreWrapper<T, U>::Contains(ConstTPtr _object) const
    {
      ConstUPtr derived = std::dynamic_pointer_cast<const U>(_object);
      return this->store->Contains(derived);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStoreWrapper<T, U>::ContainsId(unsigned int _id) const
    {
      return this->store->ContainsId(_id);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStoreWrapper<T, U>::ContainsName(const std::string &_name) const
    {
      return this->store->ContainsName(_name);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStoreWrapper<T, U>::TPtr
    BaseStoreWrapper<T, U>::GetById(unsigned int _id) const
    {
      return this->store->GetById(_id);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStoreWrapper<T, U>::TPtr
    BaseStoreWrapper<T, U>::GetByName(const std::string &_name) const
    {
      return this->store->GetByName(_name);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStoreWrapper<T, U>::TPtr
    BaseStoreWrapper<T, U>::GetByIndex(unsigned int _index) const
    {
      return this->store->GetByIndex(_index);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    bool BaseStoreWrapper<T, U>::Add(TPtr _object)
    {
      UPtr derived = std::dynamic_pointer_cast<U>(_object);
      return this->store->Add(derived);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStoreWrapper<T, U>::TPtr
    BaseStoreWrapper<T, U>::Remove(TPtr _object)
    {
      UPtr derived = std::dynamic_pointer_cast<U>(_object);
      return this->store->Remove(derived);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStoreWrapper<T, U>::TPtr
    BaseStoreWrapper<T, U>::RemoveById(unsigned int _id)
    {
      return this->store->RemoveById(_id);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStoreWrapper<T, U>::TPtr
    BaseStoreWrapper<T, U>::RemoveByName(const std::string &_name)
    {
      return this->store->RemoveByName(_name);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    typename BaseStoreWrapper<T, U>::TPtr
    BaseStoreWrapper<T, U>::RemoveByIndex(unsigned int _index)
    {
      return this->store->RemoveByIndex(_index);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStoreWrapper<T, U>::RemoveAll()
    {
      this->store->RemoveAll();
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStoreWrapper<T, U>::Destroy(TPtr _object)
    {
      UPtr derived = std::dynamic_pointer_cast<U>(_object);
      this->store->Destroy(derived);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStoreWrapper<T, U>::DestroyById(unsigned int _id)
    {
      this->store->DestroyById(_id);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStoreWrapper<T, U>::DestroyByName(const std::string &_name)
    {
      this->store->DestroyByName(_name);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStoreWrapper<T, U>::DestroyByIndex(unsigned int _index)
    {
      this->store->DestroyByIndex(_index);
    }

    //////////////////////////////////////////////////
    template <class T, class U>
    void BaseStoreWrapper<T, U>::DestroyAll()
    {
      this->store->DestroyAll();
    }
    }
  }
}
#endif
