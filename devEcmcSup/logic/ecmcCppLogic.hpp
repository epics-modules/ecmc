/*************************************************************************\
* Copyright (c) 2026 Paul Scherrer Institute
* ecmc is distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
*
*  ecmcCppLogic.hpp
*
\*************************************************************************/

#pragma once

#include "ecmcCppLogic.h"

#include <array>
#include <cctype>
#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <exception>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace ecmcCpp {

inline const ecmcCppLogicHostServices* g_hostServices = nullptr;

inline void setHostServices(const ecmcCppLogicHostServices* services) {
  g_hostServices = services;
}

inline const ecmcCppLogicHostServices* hostServices() {
  return g_hostServices;
}

template <typename T>
struct ValueType;

template <typename T, typename = void>
struct HasContiguousDataAndSize : std::false_type {};

template <typename T>
struct HasContiguousDataAndSize<
  T,
  std::void_t<
    decltype(std::declval<T&>().data()),
    decltype(std::declval<T&>().size())>> : std::true_type {};

template <typename T, typename = void>
struct HasValidateCreation : std::false_type {};

template <typename T>
struct HasValidateCreation<
  T,
  std::void_t<decltype(std::declval<T&>().validateCreation(std::declval<std::string*>()))>>
  : std::true_type {};

template <typename T>
inline int prepareAutoVectorBinding(ecmcCppLogicItemBinding* binding, uint32_t sourceBytes) {
  if (!binding || !binding->prepareContext) {
    return -1;
  }
  if ((sourceBytes % sizeof(T)) != 0u) {
    return -1;
  }

  auto* values = static_cast<std::vector<T>*>(binding->prepareContext);
  values->resize(sourceBytes / sizeof(T));
  binding->data = values->data();
  binding->bytes = sourceBytes;
  return 0;
}

template <>
struct ValueType<bool> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_BOOL;
};
template <>
struct ValueType<int8_t> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_S8;
};
template <>
struct ValueType<uint8_t> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_U8;
};
template <>
struct ValueType<int16_t> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_S16;
};
template <>
struct ValueType<uint16_t> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_U16;
};
template <>
struct ValueType<int32_t> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_S32;
};
template <>
struct ValueType<uint32_t> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_U32;
};
template <>
struct ValueType<float> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_F32;
};
template <>
struct ValueType<double> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_F64;
};
template <>
struct ValueType<uint64_t> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_U64;
};
template <>
struct ValueType<int64_t> {
  static constexpr uint32_t value = ECMC_CPP_TYPE_S64;
};

template <typename T>
inline ecmcCppLogicItemBinding ecmcInput(const char* item_name, T* value) {
  using RawT = std::remove_cv_t<T>;
  return {item_name,
          value,
          ValueType<RawT>::value,
          static_cast<uint32_t>(sizeof(RawT)),
          0u,
          ECMC_CPP_BIND_FLAG_NONE,
          nullptr,
          nullptr};
}

template <typename T>
inline ecmcCppLogicItemBinding ecmcOutput(const char* item_name, T* value) {
  using RawT = std::remove_cv_t<T>;
  return {item_name,
          value,
          ValueType<RawT>::value,
          static_cast<uint32_t>(sizeof(RawT)),
          1u,
          ECMC_CPP_BIND_FLAG_NONE,
          nullptr,
          nullptr};
}

template <typename T>
inline ecmcCppLogicExportedVar epicsReadOnly(const char* name, T* value) {
  using RawT = std::remove_cv_t<T>;
  return {name, value, ValueType<RawT>::value, static_cast<uint32_t>(sizeof(RawT)), 0u};
}

template <typename T>
inline ecmcCppLogicExportedVar epicsWritable(const char* name, T* value) {
  using RawT = std::remove_cv_t<T>;
  return {name, value, ValueType<RawT>::value, static_cast<uint32_t>(sizeof(RawT)), 1u};
}

template <typename T>
inline ecmcCppLogicItemBinding ecmcInputArray(const char* item_name,
                                                 T*          values,
                                                 size_t      count) {
  using RawT = std::remove_cv_t<T>;
  return {
    item_name,
    values,
    ValueType<RawT>::value,
    static_cast<uint32_t>(sizeof(RawT) * count),
    0u,
    ECMC_CPP_BIND_FLAG_NONE,
    nullptr,
    nullptr,
  };
}

template <typename T, size_t N>
inline ecmcCppLogicItemBinding ecmcInputArray(const char* item_name, T (&values)[N]) {
  return ecmcInputArray(item_name, values, N);
}

template <typename T, size_t N>
inline ecmcCppLogicItemBinding ecmcInputArray(const char* item_name,
                                                 std::array<T, N>& values) {
  return ecmcInputArray(item_name, values.data(), values.size());
}

template <typename Container,
          typename ValueT = std::remove_pointer_t<decltype(std::declval<Container&>().data())>,
          std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
inline ecmcCppLogicItemBinding ecmcInputArray(const char* item_name, Container& values) {
  return ecmcInputArray(item_name, values.data(), values.size());
}

template <typename T>
inline ecmcCppLogicItemBinding ecmcOutputArray(const char* item_name,
                                                  T*          values,
                                                  size_t      count) {
  using RawT = std::remove_cv_t<T>;
  return {
    item_name,
    values,
    ValueType<RawT>::value,
    static_cast<uint32_t>(sizeof(RawT) * count),
    1u,
    ECMC_CPP_BIND_FLAG_NONE,
    nullptr,
    nullptr,
  };
}

template <typename T, size_t N>
inline ecmcCppLogicItemBinding ecmcOutputArray(const char* item_name, T (&values)[N]) {
  return ecmcOutputArray(item_name, values, N);
}

template <typename T, size_t N>
inline ecmcCppLogicItemBinding ecmcOutputArray(const char* item_name,
                                                  std::array<T, N>& values) {
  return ecmcOutputArray(item_name, values.data(), values.size());
}

template <typename Container,
          typename ValueT = std::remove_pointer_t<decltype(std::declval<Container&>().data())>,
          std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
inline ecmcCppLogicItemBinding ecmcOutputArray(const char* item_name, Container& values) {
  return ecmcOutputArray(item_name, values.data(), values.size());
}

template <typename T>
inline ecmcCppLogicItemBinding ecmcInputAutoArray(const char* item_name,
                                                     std::vector<T>& values) {
  return {item_name,
          nullptr,
          ValueType<std::remove_cv_t<T>>::value,
          0u,
          0u,
          ECMC_CPP_BIND_FLAG_AUTO_SIZE,
          &prepareAutoVectorBinding<T>,
          &values};
}

template <typename T>
inline ecmcCppLogicItemBinding ecmcOutputAutoArray(const char* item_name,
                                                      std::vector<T>& values) {
  return {item_name,
          nullptr,
          ValueType<std::remove_cv_t<T>>::value,
          0u,
          1u,
          ECMC_CPP_BIND_FLAG_AUTO_SIZE,
          &prepareAutoVectorBinding<T>,
          &values};
}

inline ecmcCppLogicItemBinding ecmcInputBytes(const char* item_name,
                                                 void*       data,
                                                 uint32_t    bytes,
                                                 uint32_t    type = ECMC_CPP_TYPE_U8) {
  return {item_name, data, type, bytes, 0u, ECMC_CPP_BIND_FLAG_NONE, nullptr, nullptr};
}

inline ecmcCppLogicItemBinding ecmcOutputBytes(const char* item_name,
                                                  void*       data,
                                                  uint32_t    bytes,
                                                  uint32_t    type = ECMC_CPP_TYPE_U8) {
  return {item_name, data, type, bytes, 1u, ECMC_CPP_BIND_FLAG_NONE, nullptr, nullptr};
}

template <typename T>
inline ecmcCppLogicExportedVar epicsReadOnlyArray(const char* name,
                                                     T*          values,
                                                     size_t      count) {
  using RawT = std::remove_cv_t<T>;
  return {
    name,
    values,
    ValueType<RawT>::value,
    static_cast<uint32_t>(sizeof(RawT) * count),
    0u,
  };
}

template <typename T, size_t N>
inline ecmcCppLogicExportedVar epicsReadOnlyArray(const char* name, T (&values)[N]) {
  return epicsReadOnlyArray(name, values, N);
}

template <typename T, size_t N>
inline ecmcCppLogicExportedVar epicsReadOnlyArray(const char* name,
                                                     std::array<T, N>& values) {
  return epicsReadOnlyArray(name, values.data(), values.size());
}

template <typename Container,
          typename ValueT = std::remove_pointer_t<decltype(std::declval<Container&>().data())>,
          std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
inline ecmcCppLogicExportedVar epicsReadOnlyArray(const char* name, Container& values) {
  return epicsReadOnlyArray(name, values.data(), values.size());
}

template <typename T>
inline ecmcCppLogicExportedVar epicsWritableArray(const char* name,
                                                     T*          values,
                                                     size_t      count) {
  using RawT = std::remove_cv_t<T>;
  return {
    name,
    values,
    ValueType<RawT>::value,
    static_cast<uint32_t>(sizeof(RawT) * count),
    1u,
  };
}

template <typename T, size_t N>
inline ecmcCppLogicExportedVar epicsWritableArray(const char* name, T (&values)[N]) {
  return epicsWritableArray(name, values, N);
}

template <typename T, size_t N>
inline ecmcCppLogicExportedVar epicsWritableArray(const char* name,
                                                     std::array<T, N>& values) {
  return epicsWritableArray(name, values.data(), values.size());
}

template <typename Container,
          typename ValueT = std::remove_pointer_t<decltype(std::declval<Container&>().data())>,
          std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
inline ecmcCppLogicExportedVar epicsWritableArray(const char* name, Container& values) {
  return epicsWritableArray(name, values.data(), values.size());
}

inline ecmcCppLogicExportedVar epicsReadOnlyBytes(const char* name,
                                                     void*       data,
                                                     uint32_t    bytes,
                                                     uint32_t    type = ECMC_CPP_TYPE_U8) {
  return {name, data, type, bytes, 0u};
}

inline ecmcCppLogicExportedVar epicsWritableBytes(const char* name,
                                                     void*       data,
                                                     uint32_t    bytes,
                                                     uint32_t    type = ECMC_CPP_TYPE_U8) {
  return {name, data, type, bytes, 1u};
}

class EcmcItems {
 public:
  static const char* ownName(std::deque<std::string>* storage, std::string_view name) {
    if (!storage) {
      return "";
    }
    storage->emplace_back(name);
    return storage->back().c_str();
  }

  template <typename T>
  EcmcItems& input(const char* item_name, T& value) {
    bindings_.push_back(ecmcInput(ownName(&ownedNames_, item_name ? std::string_view(item_name)
                                                                  : std::string_view()),
                                  &value));
    return *this;
  }

  template <typename T>
  EcmcItems& input(std::string_view item_name, T& value) {
    bindings_.push_back(ecmcInput(ownName(&ownedNames_, item_name), &value));
    return *this;
  }

  template <typename T>
  EcmcItems& output(const char* item_name, T& value) {
    bindings_.push_back(ecmcOutput(ownName(&ownedNames_, item_name ? std::string_view(item_name)
                                                                   : std::string_view()),
                                   &value));
    return *this;
  }

  template <typename T>
  EcmcItems& output(std::string_view item_name, T& value) {
    bindings_.push_back(ecmcOutput(ownName(&ownedNames_, item_name), &value));
    return *this;
  }

  template <typename T>
  EcmcItems& inputArray(const char* item_name, T* values, size_t count) {
    bindings_.push_back(ecmcInputArray(
      ownName(&ownedNames_, item_name ? std::string_view(item_name) : std::string_view()),
      values,
      count));
    return *this;
  }

  template <typename T>
  EcmcItems& inputArray(std::string_view item_name, T* values, size_t count) {
    bindings_.push_back(ecmcInputArray(ownName(&ownedNames_, item_name), values, count));
    return *this;
  }

  template <typename T, size_t N>
  EcmcItems& inputArray(const char* item_name, T (&values)[N]) {
    bindings_.push_back(ecmcInputArray(item_name, values));
    return *this;
  }

  template <typename T, size_t N>
  EcmcItems& inputArray(std::string_view item_name, T (&values)[N]) {
    bindings_.push_back(ecmcInputArray(ownName(&ownedNames_, item_name), values));
    return *this;
  }

  template <typename T, size_t N>
  EcmcItems& inputArray(const char* item_name, std::array<T, N>& values) {
    bindings_.push_back(ecmcInputArray(item_name, values));
    return *this;
  }

  template <typename T, size_t N>
  EcmcItems& inputArray(std::string_view item_name, std::array<T, N>& values) {
    bindings_.push_back(ecmcInputArray(ownName(&ownedNames_, item_name), values));
    return *this;
  }

  template <typename Container,
            std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
  EcmcItems& inputArray(const char* item_name, Container& values) {
    bindings_.push_back(ecmcInputArray(item_name, values));
    return *this;
  }

  template <typename Container,
            std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
  EcmcItems& inputArray(std::string_view item_name, Container& values) {
    bindings_.push_back(ecmcInputArray(ownName(&ownedNames_, item_name), values));
    return *this;
  }

  template <typename T>
  EcmcItems& outputArray(const char* item_name, T* values, size_t count) {
    bindings_.push_back(ecmcOutputArray(
      ownName(&ownedNames_, item_name ? std::string_view(item_name) : std::string_view()),
      values,
      count));
    return *this;
  }

  template <typename T>
  EcmcItems& outputArray(std::string_view item_name, T* values, size_t count) {
    bindings_.push_back(ecmcOutputArray(ownName(&ownedNames_, item_name), values, count));
    return *this;
  }

  template <typename T, size_t N>
  EcmcItems& outputArray(const char* item_name, T (&values)[N]) {
    bindings_.push_back(ecmcOutputArray(item_name, values));
    return *this;
  }

  template <typename T, size_t N>
  EcmcItems& outputArray(std::string_view item_name, T (&values)[N]) {
    bindings_.push_back(ecmcOutputArray(ownName(&ownedNames_, item_name), values));
    return *this;
  }

  template <typename T, size_t N>
  EcmcItems& outputArray(const char* item_name, std::array<T, N>& values) {
    bindings_.push_back(ecmcOutputArray(item_name, values));
    return *this;
  }

  template <typename T, size_t N>
  EcmcItems& outputArray(std::string_view item_name, std::array<T, N>& values) {
    bindings_.push_back(ecmcOutputArray(ownName(&ownedNames_, item_name), values));
    return *this;
  }

  template <typename Container,
            std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
  EcmcItems& outputArray(const char* item_name, Container& values) {
    bindings_.push_back(ecmcOutputArray(item_name, values));
    return *this;
  }

  template <typename Container,
            std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
  EcmcItems& outputArray(std::string_view item_name, Container& values) {
    bindings_.push_back(ecmcOutputArray(ownName(&ownedNames_, item_name), values));
    return *this;
  }

  template <typename T>
  EcmcItems& inputAutoArray(const char* item_name, std::vector<T>& values) {
    bindings_.push_back(
      ecmcInputAutoArray(ownName(&ownedNames_,
                                 item_name ? std::string_view(item_name) : std::string_view()),
                         values));
    return *this;
  }

  template <typename T>
  EcmcItems& inputAutoArray(std::string_view item_name, std::vector<T>& values) {
    bindings_.push_back(ecmcInputAutoArray(ownName(&ownedNames_, item_name), values));
    return *this;
  }

  template <typename T>
  EcmcItems& outputAutoArray(const char* item_name, std::vector<T>& values) {
    bindings_.push_back(
      ecmcOutputAutoArray(ownName(&ownedNames_,
                                  item_name ? std::string_view(item_name) : std::string_view()),
                          values));
    return *this;
  }

  template <typename T>
  EcmcItems& outputAutoArray(std::string_view item_name, std::vector<T>& values) {
    bindings_.push_back(ecmcOutputAutoArray(ownName(&ownedNames_, item_name), values));
    return *this;
  }

  EcmcItems& inputBytes(const char* item_name,
                        void*       data,
                        uint32_t    bytes,
                        uint32_t    type = ECMC_CPP_TYPE_U8) {
    bindings_.push_back(ecmcInputBytes(
      ownName(&ownedNames_, item_name ? std::string_view(item_name) : std::string_view()),
      data,
      bytes,
      type));
    return *this;
  }

  EcmcItems& inputBytes(std::string_view item_name,
                        void*            data,
                        uint32_t         bytes,
                        uint32_t         type = ECMC_CPP_TYPE_U8) {
    bindings_.push_back(ecmcInputBytes(ownName(&ownedNames_, item_name), data, bytes, type));
    return *this;
  }

  EcmcItems& outputBytes(const char* item_name,
                         void*       data,
                         uint32_t    bytes,
                         uint32_t    type = ECMC_CPP_TYPE_U8) {
    bindings_.push_back(ecmcOutputBytes(
      ownName(&ownedNames_, item_name ? std::string_view(item_name) : std::string_view()),
      data,
      bytes,
      type));
    return *this;
  }

  EcmcItems& outputBytes(std::string_view item_name,
                         void*            data,
                         uint32_t         bytes,
                         uint32_t         type = ECMC_CPP_TYPE_U8) {
    bindings_.push_back(ecmcOutputBytes(ownName(&ownedNames_, item_name), data, bytes, type));
    return *this;
  }

  const ecmcCppLogicItemBinding* data() const {
    return bindings_.data();
  }

  uint32_t count() const {
    return static_cast<uint32_t>(bindings_.size());
  }

 private:
  std::deque<std::string> ownedNames_;
  std::vector<ecmcCppLogicItemBinding> bindings_;
};

class EpicsExports {
 public:
  static const char* ownName(std::deque<std::string>* storage, std::string_view name) {
    if (!storage) {
      return "";
    }
    storage->emplace_back(name);
    return storage->back().c_str();
  }

  template <typename T>
  EpicsExports& readOnly(const char* name, T& value) {
    exports_.push_back(epicsReadOnly(ownName(&ownedNames_,
                                             name ? std::string_view(name) : std::string_view()),
                                     &value));
    return *this;
  }

  template <typename T>
  EpicsExports& readOnly(std::string_view name, T& value) {
    exports_.push_back(epicsReadOnly(ownName(&ownedNames_, name), &value));
    return *this;
  }

  template <typename T>
  EpicsExports& writable(const char* name, T& value) {
    exports_.push_back(epicsWritable(ownName(&ownedNames_,
                                             name ? std::string_view(name) : std::string_view()),
                                     &value));
    return *this;
  }

  template <typename T>
  EpicsExports& writable(std::string_view name, T& value) {
    exports_.push_back(epicsWritable(ownName(&ownedNames_, name), &value));
    return *this;
  }

  template <typename T>
  EpicsExports& readOnlyArray(const char* name, T* values, size_t count) {
    exports_.push_back(epicsReadOnlyArray(
      ownName(&ownedNames_, name ? std::string_view(name) : std::string_view()),
      values,
      count));
    return *this;
  }

  template <typename T>
  EpicsExports& readOnlyArray(std::string_view name, T* values, size_t count) {
    exports_.push_back(epicsReadOnlyArray(ownName(&ownedNames_, name), values, count));
    return *this;
  }

  template <typename T, size_t N>
  EpicsExports& readOnlyArray(const char* name, T (&values)[N]) {
    exports_.push_back(epicsReadOnlyArray(name, values));
    return *this;
  }

  template <typename T, size_t N>
  EpicsExports& readOnlyArray(std::string_view name, T (&values)[N]) {
    exports_.push_back(epicsReadOnlyArray(ownName(&ownedNames_, name), values));
    return *this;
  }

  template <typename T, size_t N>
  EpicsExports& readOnlyArray(const char* name, std::array<T, N>& values) {
    exports_.push_back(epicsReadOnlyArray(name, values));
    return *this;
  }

  template <typename T, size_t N>
  EpicsExports& readOnlyArray(std::string_view name, std::array<T, N>& values) {
    exports_.push_back(epicsReadOnlyArray(ownName(&ownedNames_, name), values));
    return *this;
  }

  template <typename Container,
            std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
  EpicsExports& readOnlyArray(const char* name, Container& values) {
    exports_.push_back(epicsReadOnlyArray(name, values));
    return *this;
  }

  template <typename Container,
            std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
  EpicsExports& readOnlyArray(std::string_view name, Container& values) {
    exports_.push_back(epicsReadOnlyArray(ownName(&ownedNames_, name), values));
    return *this;
  }

  template <typename T>
  EpicsExports& writableArray(const char* name, T* values, size_t count) {
    exports_.push_back(epicsWritableArray(
      ownName(&ownedNames_, name ? std::string_view(name) : std::string_view()),
      values,
      count));
    return *this;
  }

  template <typename T>
  EpicsExports& writableArray(std::string_view name, T* values, size_t count) {
    exports_.push_back(epicsWritableArray(ownName(&ownedNames_, name), values, count));
    return *this;
  }

  template <typename T, size_t N>
  EpicsExports& writableArray(const char* name, T (&values)[N]) {
    exports_.push_back(epicsWritableArray(name, values));
    return *this;
  }

  template <typename T, size_t N>
  EpicsExports& writableArray(std::string_view name, T (&values)[N]) {
    exports_.push_back(epicsWritableArray(ownName(&ownedNames_, name), values));
    return *this;
  }

  template <typename T, size_t N>
  EpicsExports& writableArray(const char* name, std::array<T, N>& values) {
    exports_.push_back(epicsWritableArray(name, values));
    return *this;
  }

  template <typename T, size_t N>
  EpicsExports& writableArray(std::string_view name, std::array<T, N>& values) {
    exports_.push_back(epicsWritableArray(ownName(&ownedNames_, name), values));
    return *this;
  }

  template <typename Container,
            std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
  EpicsExports& writableArray(const char* name, Container& values) {
    exports_.push_back(epicsWritableArray(name, values));
    return *this;
  }

  template <typename Container,
            std::enable_if_t<HasContiguousDataAndSize<Container>::value, int> = 0>
  EpicsExports& writableArray(std::string_view name, Container& values) {
    exports_.push_back(epicsWritableArray(ownName(&ownedNames_, name), values));
    return *this;
  }

  EpicsExports& readOnlyBytes(const char* name,
                              void*       data,
                              uint32_t    bytes,
                              uint32_t    type = ECMC_CPP_TYPE_U8) {
    exports_.push_back(epicsReadOnlyBytes(
      ownName(&ownedNames_, name ? std::string_view(name) : std::string_view()),
      data,
      bytes,
      type));
    return *this;
  }

  EpicsExports& readOnlyBytes(std::string_view name,
                              void*            data,
                              uint32_t         bytes,
                              uint32_t         type = ECMC_CPP_TYPE_U8) {
    exports_.push_back(epicsReadOnlyBytes(ownName(&ownedNames_, name), data, bytes, type));
    return *this;
  }

  EpicsExports& writableBytes(const char* name,
                              void*       data,
                              uint32_t    bytes,
                              uint32_t    type = ECMC_CPP_TYPE_U8) {
    exports_.push_back(epicsWritableBytes(
      ownName(&ownedNames_, name ? std::string_view(name) : std::string_view()),
      data,
      bytes,
      type));
    return *this;
  }

  EpicsExports& writableBytes(std::string_view name,
                              void*            data,
                              uint32_t         bytes,
                              uint32_t         type = ECMC_CPP_TYPE_U8) {
    exports_.push_back(epicsWritableBytes(ownName(&ownedNames_, name), data, bytes, type));
    return *this;
  }

  const ecmcCppLogicExportedVar* data() const {
    return exports_.data();
  }

  uint32_t count() const {
    return static_cast<uint32_t>(exports_.size());
  }

 private:
  std::deque<std::string> ownedNames_;
  std::vector<ecmcCppLogicExportedVar> exports_;
};

class LogicBase {
 public:
  virtual ~LogicBase() = default;
  virtual void enterRealtime() {}
  virtual void exitRealtime() {}
  virtual void run() = 0;

  EcmcItems ecmc;
  EpicsExports epics;
};

inline double getCycleTimeS() {
  return (g_hostServices && g_hostServices->get_cycle_time_s)
           ? g_hostServices->get_cycle_time_s()
           : 0.0;
}

inline uint64_t getEcTimeNs() {
  return (g_hostServices && g_hostServices->get_ec_time_ns)
           ? g_hostServices->get_ec_time_ns()
           : 0u;
}

inline uint64_t getEcTimeOffsetNs() {
  return (g_hostServices && g_hostServices->get_ec_time_offset_ns)
           ? g_hostServices->get_ec_time_offset_ns()
           : 0u;
}

inline uint64_t getEcLastReceiveTimeNs() {
  return (g_hostServices && g_hostServices->get_ec_last_receive_time_ns)
           ? g_hostServices->get_ec_last_receive_time_ns()
           : 0u;
}

inline uint64_t getEcLastSendTimeNs() {
  return (g_hostServices && g_hostServices->get_ec_last_send_time_ns)
           ? g_hostServices->get_ec_last_send_time_ns()
           : 0u;
}

inline int32_t getEcDomainState(int32_t domain_index) {
  return (g_hostServices && g_hostServices->get_ec_domain_state)
           ? g_hostServices->get_ec_domain_state(domain_index)
           : -1;
}

inline int32_t getEcStatusOK() {
  return (g_hostServices && g_hostServices->get_ec_status_ok)
           ? g_hostServices->get_ec_status_ok()
           : 0;
}

inline uint32_t getEcMasterStateWord(int32_t master_index = -1) {
  return (g_hostServices && g_hostServices->get_ec_master_state_word)
           ? g_hostServices->get_ec_master_state_word(master_index)
           : 0u;
}

inline uint32_t getEcSlaveStateWord(int32_t slave_index, int32_t master_index = -1) {
  return (g_hostServices && g_hostServices->get_ec_slave_state_word)
           ? g_hostServices->get_ec_slave_state_word(master_index, slave_index)
           : 0u;
}

inline int32_t axisUseInternalTraj(int32_t axis_index) {
  return (g_hostServices && g_hostServices->set_axis_traj_source)
           ? g_hostServices->set_axis_traj_source(axis_index, 0)
           : -1;
}

inline int32_t axisUseExternalTraj(int32_t axis_index) {
  return (g_hostServices && g_hostServices->set_axis_traj_source)
           ? g_hostServices->set_axis_traj_source(axis_index, 1)
           : -1;
}

inline int32_t axisUseInternalEnc(int32_t axis_index) {
  return (g_hostServices && g_hostServices->set_axis_enc_source)
           ? g_hostServices->set_axis_enc_source(axis_index, 0)
           : -1;
}

inline int32_t axisUseExternalEnc(int32_t axis_index) {
  return (g_hostServices && g_hostServices->set_axis_enc_source)
           ? g_hostServices->set_axis_enc_source(axis_index, 1)
           : -1;
}

inline double axisGetActualPos(int32_t axis_index) {
  return (g_hostServices && g_hostServices->get_axis_actual_pos)
           ? g_hostServices->get_axis_actual_pos(axis_index)
           : 0.0;
}

inline double axisGetSetpointPos(int32_t axis_index) {
  return (g_hostServices && g_hostServices->get_axis_setpoint_pos)
           ? g_hostServices->get_axis_setpoint_pos(axis_index)
           : 0.0;
}

inline double axisGetActualVel(int32_t axis_index) {
  return (g_hostServices && g_hostServices->get_axis_actual_vel)
           ? g_hostServices->get_axis_actual_vel(axis_index)
           : 0.0;
}

inline double axisGetSetpointVel(int32_t axis_index) {
  return (g_hostServices && g_hostServices->get_axis_setpoint_vel)
           ? g_hostServices->get_axis_setpoint_vel(axis_index)
           : 0.0;
}

inline int32_t axisIsEnabled(int32_t axis_index) {
  return (g_hostServices && g_hostServices->get_axis_enabled)
           ? g_hostServices->get_axis_enabled(axis_index)
           : 0;
}

inline int32_t axisIsBusy(int32_t axis_index) {
  return (g_hostServices && g_hostServices->get_axis_busy)
           ? g_hostServices->get_axis_busy(axis_index)
           : 0;
}

inline int32_t axisHasError(int32_t axis_index) {
  return (g_hostServices && g_hostServices->get_axis_error)
           ? g_hostServices->get_axis_error(axis_index)
           : 0;
}

inline int32_t axisGetErrorId(int32_t axis_index) {
  return (g_hostServices && g_hostServices->get_axis_error_id)
           ? g_hostServices->get_axis_error_id(axis_index)
           : 0;
}

inline int32_t axisSetExternalSetpointPos(int32_t axis_index, double value) {
  return (g_hostServices && g_hostServices->set_axis_ext_set_pos)
           ? g_hostServices->set_axis_ext_set_pos(axis_index, value)
           : -1;
}

inline int32_t axisSetExternalEncoderPos(int32_t axis_index, double value) {
  return (g_hostServices && g_hostServices->set_axis_ext_act_pos)
           ? g_hostServices->set_axis_ext_act_pos(axis_index, value)
           : -1;
}

inline double axisGetEncoderActualPos(int32_t axis_index, int32_t encoder_index) {
  return (g_hostServices && g_hostServices->get_axis_encoder_actual_pos)
           ? g_hostServices->get_axis_encoder_actual_pos(axis_index, encoder_index)
           : 0.0;
}

inline int32_t axisSetEncoderActualPos(int32_t axis_index,
                                       int32_t encoder_index,
                                       double value) {
  return (g_hostServices && g_hostServices->set_axis_encoder_actual_pos)
           ? g_hostServices->set_axis_encoder_actual_pos(axis_index, encoder_index, value)
           : -1;
}

inline int32_t axisGetEncoderHomed(int32_t axis_index, int32_t encoder_index) {
  return (g_hostServices && g_hostServices->get_axis_encoder_homed)
           ? g_hostServices->get_axis_encoder_homed(axis_index, encoder_index)
           : -1;
}

inline int32_t axisSetEncoderHomed(int32_t axis_index,
                                   int32_t encoder_index,
                                   bool homed) {
  return (g_hostServices && g_hostServices->set_axis_encoder_homed)
           ? g_hostServices->set_axis_encoder_homed(axis_index,
                                                    encoder_index,
                                                    homed ? 1 : 0)
           : -1;
}

inline int32_t axisGetEncoderReady(int32_t axis_index, int32_t encoder_index) {
  return (g_hostServices && g_hostServices->get_axis_encoder_ready)
           ? g_hostServices->get_axis_encoder_ready(axis_index, encoder_index)
           : -1;
}

inline int32_t axisGetPrimaryEncoder(int32_t axis_index) {
  return (g_hostServices && g_hostServices->get_axis_primary_encoder)
           ? g_hostServices->get_axis_primary_encoder(axis_index)
           : -1;
}

inline int32_t axisSelectPrimaryEncoder(int32_t axis_index, int32_t encoder_index) {
  return (g_hostServices && g_hostServices->select_axis_primary_encoder)
           ? g_hostServices->select_axis_primary_encoder(axis_index, encoder_index)
           : -1;
}

using AxisStatus = ecmcCppAxisStatus;

inline int32_t axisGetStatus(int32_t axis_index, AxisStatus& status) {
  return (g_hostServices && g_hostServices->get_axis_status)
           ? g_hostServices->get_axis_status(axis_index, &status)
           : -1;
}

inline AxisStatus axisGetStatus(int32_t axis_index) {
  AxisStatus status {};
  axisGetStatus(axis_index, status);
  return status;
}

inline double lutGetValue(int32_t lut_index, double index) {
  return (g_hostServices && g_hostServices->get_lut_value)
           ? g_hostServices->get_lut_value(lut_index, index)
           : 0.0;
}

inline bool lutExists(int32_t lut_index) {
  return (g_hostServices && g_hostServices->lut_exists)
           ? g_hostServices->lut_exists(lut_index) != 0
           : false;
}

inline int32_t m2mWrite(int32_t shm_index, double value) {
  return (g_hostServices && g_hostServices->m2m_write)
           ? g_hostServices->m2m_write(shm_index, value)
           : -1;
}

inline double m2mRead(int32_t shm_index) {
  return (g_hostServices && g_hostServices->m2m_read)
           ? g_hostServices->m2m_read(shm_index)
           : 0.0;
}

inline int32_t m2mStatus() {
  return (g_hostServices && g_hostServices->m2m_status)
           ? g_hostServices->m2m_status()
           : -1;
}

inline int32_t m2mResetError() {
  return (g_hostServices && g_hostServices->m2m_reset_error)
           ? g_hostServices->m2m_reset_error()
           : -1;
}

inline int32_t m2mGetError() {
  return (g_hostServices && g_hostServices->m2m_get_error)
           ? g_hostServices->m2m_get_error()
           : -1;
}

inline int32_t m2mIocRun(int32_t master_index) {
  return (g_hostServices && g_hostServices->m2m_ioc_run)
           ? g_hostServices->m2m_ioc_run(master_index)
           : -1;
}

inline int32_t m2mIocEcOk(int32_t master_index) {
  return (g_hostServices && g_hostServices->m2m_ioc_ec_ok)
           ? g_hostServices->m2m_ioc_ec_ok(master_index)
           : -1;
}

inline int32_t dataStorageClear(int32_t storage_index) {
  return (g_hostServices && g_hostServices->ds_clear)
           ? g_hostServices->ds_clear(storage_index)
           : -1;
}

inline int32_t dataStorageAppend(int32_t storage_index, double value) {
  return (g_hostServices && g_hostServices->ds_append)
           ? g_hostServices->ds_append(storage_index, value)
           : -1;
}

inline double dataStorageGet(int32_t storage_index, int32_t data_index) {
  return (g_hostServices && g_hostServices->ds_get)
           ? g_hostServices->ds_get(storage_index, data_index)
           : 0.0;
}

inline int32_t dataStorageSet(int32_t storage_index, int32_t data_index, double value) {
  return (g_hostServices && g_hostServices->ds_set)
           ? g_hostServices->ds_set(storage_index, data_index, value)
           : -1;
}

inline int32_t dataStorageGetIndex(int32_t storage_index) {
  return (g_hostServices && g_hostServices->ds_get_index)
           ? g_hostServices->ds_get_index(storage_index)
           : -1;
}

inline int32_t dataStorageSetIndex(int32_t storage_index, int32_t data_index) {
  return (g_hostServices && g_hostServices->ds_set_index)
           ? g_hostServices->ds_set_index(storage_index, data_index)
           : -1;
}

inline int32_t dataStorageGetError() {
  return (g_hostServices && g_hostServices->ds_get_error)
           ? g_hostServices->ds_get_error()
           : -1;
}

inline int32_t dataStorageResetError() {
  return (g_hostServices && g_hostServices->ds_reset_error)
           ? g_hostServices->ds_reset_error()
           : -1;
}

inline int32_t dataStorageIsFull(int32_t storage_index) {
  return (g_hostServices && g_hostServices->ds_is_full)
           ? g_hostServices->ds_is_full(storage_index)
           : -1;
}

inline int32_t dataStorageGetSize(int32_t storage_index) {
  return (g_hostServices && g_hostServices->ds_get_size)
           ? g_hostServices->ds_get_size(storage_index)
           : -1;
}

inline int32_t dataStoragePushAsyn(int32_t storage_index) {
  return (g_hostServices && g_hostServices->ds_push_asyn)
           ? g_hostServices->ds_push_asyn(storage_index)
           : -1;
}

inline double dataStorageGetAvg(int32_t storage_index) {
  return (g_hostServices && g_hostServices->ds_get_avg)
           ? g_hostServices->ds_get_avg(storage_index)
           : 0.0;
}

inline double dataStorageGetMin(int32_t storage_index) {
  return (g_hostServices && g_hostServices->ds_get_min)
           ? g_hostServices->ds_get_min(storage_index)
           : 0.0;
}

inline double dataStorageGetMax(int32_t storage_index) {
  return (g_hostServices && g_hostServices->ds_get_max)
           ? g_hostServices->ds_get_max(storage_index)
           : 0.0;
}

inline int32_t dataStorageRead(int32_t storage_index, std::vector<double>& values) {
  if (!g_hostServices || !g_hostServices->ds_read) {
    return -1;
  }

  uint32_t size = 0;
  int32_t error = g_hostServices->ds_read(storage_index, nullptr, 0u, &size);
  if (error) {
    return error;
  }

  values.resize(size);
  if (size == 0u) {
    return 0;
  }

  return g_hostServices->ds_read(storage_index, values.data(), size, &size);
}

inline int32_t dataStorageWrite(int32_t storage_index, const double* data, uint32_t size) {
  return (g_hostServices && g_hostServices->ds_write)
           ? g_hostServices->ds_write(storage_index, data, size)
           : -1;
}

inline int32_t dataStorageWrite(int32_t storage_index, const std::vector<double>& values) {
  return dataStorageWrite(storage_index,
                          values.empty() ? nullptr : values.data(),
                          static_cast<uint32_t>(values.size()));
}

inline int32_t dataStorageAppend(int32_t storage_index, const double* data, uint32_t size) {
  return (g_hostServices && g_hostServices->ds_append_array)
           ? g_hostServices->ds_append_array(storage_index, data, size)
           : -1;
}

inline int32_t dataStorageAppend(int32_t storage_index, const std::vector<double>& values) {
  return dataStorageAppend(storage_index,
                           values.empty() ? nullptr : values.data(),
                           static_cast<uint32_t>(values.size()));
}

inline int32_t dataStorageAppendFromStorage(int32_t from_storage_index,
                                            int32_t from_data_index,
                                            uint32_t elements,
                                            int32_t to_storage_index) {
  return (g_hostServices && g_hostServices->ds_append_from_storage)
           ? g_hostServices->ds_append_from_storage(from_storage_index,
                                                    from_data_index,
                                                    elements,
                                                    to_storage_index)
           : -1;
}

inline int32_t axisGroupGetEnable(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_enable)
           ? g_hostServices->axis_group_get_enable(group_index)
           : -1;
}

inline int32_t axisGroupGetAnyEnable(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_any_enable)
           ? g_hostServices->axis_group_get_any_enable(group_index)
           : -1;
}

inline int32_t axisGroupGetEnabled(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_enabled)
           ? g_hostServices->axis_group_get_enabled(group_index)
           : -1;
}

inline int32_t axisGroupGetAnyEnabled(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_any_enabled)
           ? g_hostServices->axis_group_get_any_enabled(group_index)
           : -1;
}

inline int32_t axisGroupGetBusy(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_busy)
           ? g_hostServices->axis_group_get_busy(group_index)
           : -1;
}

inline int32_t axisGroupGetAnyBusy(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_any_busy)
           ? g_hostServices->axis_group_get_any_busy(group_index)
           : -1;
}

inline int32_t axisGroupGetAnyErrorId(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_any_error_id)
           ? g_hostServices->axis_group_get_any_error_id(group_index)
           : -1;
}

inline int32_t axisGroupSetEnable(int32_t group_index, bool enable) {
  return (g_hostServices && g_hostServices->axis_group_set_enable)
           ? g_hostServices->axis_group_set_enable(group_index, enable ? 1 : 0)
           : -1;
}

inline int32_t axisGroupSetTrajSource(int32_t group_index, int32_t source) {
  return (g_hostServices && g_hostServices->axis_group_set_traj_source)
           ? g_hostServices->axis_group_set_traj_source(group_index, source)
           : -1;
}

inline int32_t axisGroupSetEncSource(int32_t group_index, int32_t source) {
  return (g_hostServices && g_hostServices->axis_group_set_enc_source)
           ? g_hostServices->axis_group_set_enc_source(group_index, source)
           : -1;
}

inline int32_t axisGroupResetError(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_reset_error)
           ? g_hostServices->axis_group_reset_error(group_index)
           : -1;
}

inline int32_t axisGroupSetError(int32_t group_index, int32_t error_id) {
  return (g_hostServices && g_hostServices->axis_group_set_error)
           ? g_hostServices->axis_group_set_error(group_index, error_id)
           : -1;
}

inline int32_t axisGroupSetSlavedAxisInError(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_set_slaved_axis_in_error)
           ? g_hostServices->axis_group_set_slaved_axis_in_error(group_index)
           : -1;
}

inline int32_t axisGroupHalt(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_halt)
           ? g_hostServices->axis_group_halt(group_index)
           : -1;
}

inline int32_t axisGroupAxisInGroup(int32_t group_index, int32_t axis_index) {
  return (g_hostServices && g_hostServices->axis_group_axis_in_group)
           ? g_hostServices->axis_group_axis_in_group(group_index, axis_index)
           : -1;
}

inline int32_t axisGroupSize(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_size)
           ? g_hostServices->axis_group_size(group_index)
           : -1;
}

inline int32_t axisGroupGetTrajSourceExt(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_traj_source_ext)
           ? g_hostServices->axis_group_get_traj_source_ext(group_index)
           : -1;
}

inline int32_t axisGroupGetAnyTrajSourceExt(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_any_traj_source_ext)
           ? g_hostServices->axis_group_get_any_traj_source_ext(group_index)
           : -1;
}

inline int32_t axisGroupSetAllowSourceChangeWhenEnabled(int32_t group_index,
                                                        bool allow) {
  return (g_hostServices &&
          g_hostServices->axis_group_set_allow_source_change_when_enabled)
           ? g_hostServices->axis_group_set_allow_source_change_when_enabled(
               group_index,
               allow ? 1 : 0)
           : -1;
}

inline int32_t axisGroupSetMrSync(int32_t group_index, bool sync) {
  return (g_hostServices && g_hostServices->axis_group_set_mr_sync)
           ? g_hostServices->axis_group_set_mr_sync(group_index, sync ? 1 : 0)
           : -1;
}

inline int32_t axisGroupSetMrStop(int32_t group_index, bool stop) {
  return (g_hostServices && g_hostServices->axis_group_set_mr_stop)
           ? g_hostServices->axis_group_set_mr_stop(group_index, stop ? 1 : 0)
           : -1;
}

inline int32_t axisGroupSetMrCnen(int32_t group_index, bool enable) {
  return (g_hostServices && g_hostServices->axis_group_set_mr_cnen)
           ? g_hostServices->axis_group_set_mr_cnen(group_index, enable ? 1 : 0)
           : -1;
}

inline int32_t axisGroupSetAutoEnable(int32_t group_index, bool enable) {
  return (g_hostServices && g_hostServices->axis_group_set_auto_enable)
           ? g_hostServices->axis_group_set_auto_enable(group_index, enable ? 1 : 0)
           : -1;
}

inline int32_t axisGroupSetAutoDisable(int32_t group_index, bool enable) {
  return (g_hostServices && g_hostServices->axis_group_set_auto_disable)
           ? g_hostServices->axis_group_set_auto_disable(group_index, enable ? 1 : 0)
           : -1;
}

inline int32_t axisGroupSetCtrlWithinDeadband(int32_t group_index, bool within) {
  return (g_hostServices && g_hostServices->axis_group_set_ctrl_within_deadband)
           ? g_hostServices->axis_group_set_ctrl_within_deadband(group_index,
                                                                 within ? 1 : 0)
           : -1;
}

inline int32_t axisGroupSetIgnoreMrStatusCheckAtDisable(int32_t group_index,
                                                        bool ignore) {
  return (g_hostServices &&
          g_hostServices->axis_group_set_ignore_mr_status_check_at_disable)
           ? g_hostServices->axis_group_set_ignore_mr_status_check_at_disable(
               group_index,
               ignore ? 1 : 0)
           : -1;
}

inline int32_t axisGroupGetAnyAtForwardLimit(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_any_at_forward_limit)
           ? g_hostServices->axis_group_get_any_at_forward_limit(group_index)
           : -1;
}

inline int32_t axisGroupGetAnyAtBackwardLimit(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_any_at_backward_limit)
           ? g_hostServices->axis_group_get_any_at_backward_limit(group_index)
           : -1;
}

inline int32_t axisGroupGetAnyAtLimit(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_any_at_limit)
           ? g_hostServices->axis_group_get_any_at_limit(group_index)
           : -1;
}

inline int32_t axisGroupGetAnyInterlocked(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_any_interlocked)
           ? g_hostServices->axis_group_get_any_interlocked(group_index)
           : -1;
}

inline int32_t axisGroupSetSlavedAxisInterlocked(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_set_slaved_axis_interlocked)
           ? g_hostServices->axis_group_set_slaved_axis_interlocked(group_index)
           : -1;
}

inline int32_t axisGroupGetCtrlWithinDeadband(int32_t group_index) {
  return (g_hostServices && g_hostServices->axis_group_get_ctrl_within_deadband)
           ? g_hostServices->axis_group_get_ctrl_within_deadband(group_index)
           : -1;
}

inline int32_t requestIocExit(int32_t exit_code = 0) {
  return (g_hostServices && g_hostServices->request_ioc_exit)
           ? g_hostServices->request_ioc_exit(exit_code)
           : -1;
}

inline int32_t setCreateErrorMessage(const char* message) {
  return (g_hostServices && g_hostServices->set_create_error_message)
           ? g_hostServices->set_create_error_message(message)
           : -1;
}

inline int32_t setCreateErrorMessage(const std::string& message) {
  return setCreateErrorMessage(message.c_str());
}

inline int32_t setEnableDbg(bool enable) {
  return (g_hostServices && g_hostServices->set_enable_dbg)
           ? g_hostServices->set_enable_dbg(enable ? 1 : 0)
           : -1;
}

inline int32_t getIocState() {
  return (g_hostServices && g_hostServices->get_ioc_state)
           ? g_hostServices->get_ioc_state()
           : -1;
}

inline std::string getMacrosString() {
  if (!g_hostServices || !g_hostServices->get_macros_text) {
    return {};
  }

  const char* text = g_hostServices->get_macros_text();
  return text ? std::string(text) : std::string();
}

inline std::string getMacroValue(const std::string& macrosString, const std::string& key) {
  auto trimCopy = [](const std::string& value) -> std::string {
    size_t begin = 0u;
    while (begin < value.size() &&
           std::isspace(static_cast<unsigned char>(value[begin])) != 0) {
      ++begin;
    }

    size_t end = value.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(value[end - 1u])) != 0) {
      --end;
    }

    return value.substr(begin, end - begin);
  };

  auto stripOptionalQuotes = [](const std::string& value) -> std::string {
    if (value.size() >= 2u &&
        ((value.front() == '\'' && value.back() == '\'') ||
         (value.front() == '"' && value.back() == '"'))) {
      return value.substr(1u, value.size() - 2u);
    }
    return value;
  };

  auto splitMacroTokens = [](const std::string& text) -> std::vector<std::string> {
    std::vector<std::string> tokens;
    std::string current;
    char quoteChar = '\0';

    for (char c : text) {
      if (c == '\'' || c == '"') {
        if (quoteChar == '\0') {
          quoteChar = c;
        } else if (quoteChar == c) {
          quoteChar = '\0';
        }
        current.push_back(c);
        continue;
      }

      if (c == ',' && quoteChar == '\0') {
        tokens.emplace_back(std::move(current));
        current.clear();
        continue;
      }

      current.push_back(c);
    }

    tokens.emplace_back(std::move(current));
    return tokens;
  };

  const std::string wantedKey = trimCopy(key);
  if (wantedKey.empty()) {
    return {};
  }

  for (const std::string& token : splitMacroTokens(macrosString)) {
    if (token.empty()) {
      continue;
    }

    const size_t equals = token.find('=');
    if (equals == std::string::npos) {
      continue;
    }

    const std::string tokenKey = trimCopy(token.substr(0, equals));
    if (tokenKey != wantedKey) {
      continue;
    }

    return stripOptionalQuotes(trimCopy(token.substr(equals + 1u)));
  }

  return {};
}

inline std::string getMacroValue(const char* macrosString, const char* key) {
  return getMacroValue(macrosString ? std::string(macrosString) : std::string(),
                       key ? std::string(key) : std::string());
}

inline std::string getMacroValueString(const std::string& macrosString,
                                       const std::string& key,
                                       const std::string& defaultValue = std::string()) {
  const std::string value = getMacroValue(macrosString, key);
  return value.empty() ? defaultValue : value;
}

inline std::string getMacroValueString(const char* macrosString,
                                       const char* key,
                                       const char* defaultValue = "") {
  const std::string value = getMacroValue(macrosString, key);
  return value.empty() ? std::string(defaultValue ? defaultValue : "") : value;
}

inline int getMacroValueInt(const std::string& macrosString,
                            const std::string& key,
                            int defaultValue = 0) {
  const std::string value = getMacroValue(macrosString, key);
  if (value.empty()) {
    return defaultValue;
  }

  char* endPtr = nullptr;
  const long parsed = std::strtol(value.c_str(), &endPtr, 0);
  if (!endPtr || *endPtr != '\0') {
    return defaultValue;
  }
  return static_cast<int>(parsed);
}

inline int getMacroValueInt(const char* macrosString,
                            const char* key,
                            int defaultValue = 0) {
  return getMacroValueInt(macrosString ? std::string(macrosString) : std::string(),
                          key ? std::string(key) : std::string(),
                          defaultValue);
}

inline double getMacroValueDouble(const std::string& macrosString,
                                  const std::string& key,
                                  double defaultValue = 0.0) {
  const std::string value = getMacroValue(macrosString, key);
  if (value.empty()) {
    return defaultValue;
  }

  char* endPtr = nullptr;
  const double parsed = std::strtod(value.c_str(), &endPtr);
  if (!endPtr || *endPtr != '\0') {
    return defaultValue;
  }
  return parsed;
}

inline double getMacroValueDouble(const char* macrosString,
                                  const char* key,
                                  double defaultValue = 0.0) {
  return getMacroValueDouble(macrosString ? std::string(macrosString) : std::string(),
                             key ? std::string(key) : std::string(),
                             defaultValue);
}

inline void publishDebugText(const char* message) {
  if (g_hostServices && g_hostServices->publish_debug_text) {
    g_hostServices->publish_debug_text(message);
  }
}

inline void publishDebugText(const std::string& message) {
  publishDebugText(message.c_str());
}

namespace detail {

template <typename LogicT>
inline void setHostServicesAdapter(const ecmcCppLogicHostServices* services) {
  static_assert(std::is_base_of_v<LogicBase, LogicT>,
                "LogicT must derive from ecmcCpp::LogicBase");
  ecmcCpp::setHostServices(services);
}

template <typename LogicT>
inline int32_t createInstanceAdapter(void** instance) {
  static_assert(std::is_base_of_v<LogicBase, LogicT>,
                "LogicT must derive from ecmcCpp::LogicBase");
  if (!instance) {
    return ECMC_CPP_LOGIC_CREATE_INSTANCE_FAIL;
  }
  *instance = nullptr;
  try {
    *instance = new LogicT();
    if constexpr (HasValidateCreation<LogicT>::value) {
      std::string errorMessage;
      const int32_t error = static_cast<LogicT*>(*instance)->validateCreation(&errorMessage);
      if (error != 0) {
        if (!errorMessage.empty()) {
          setCreateErrorMessage(errorMessage);
        }
        delete static_cast<LogicT*>(*instance);
        *instance = nullptr;
        return error;
      }
    }
  } catch (const std::exception& e) {
    setCreateErrorMessage(e.what());
    delete static_cast<LogicT*>(*instance);
    *instance = nullptr;
    return ECMC_CPP_LOGIC_CREATE_INSTANCE_FAIL;
  } catch (...) {
    setCreateErrorMessage("unknown exception during C++ logic construction");
    delete static_cast<LogicT*>(*instance);
    *instance = nullptr;
    return ECMC_CPP_LOGIC_CREATE_INSTANCE_FAIL;
  }
  return *instance ? 0 : ECMC_CPP_LOGIC_CREATE_INSTANCE_FAIL;
}

template <typename LogicT>
inline void destroyInstanceAdapter(void* instance) {
  delete static_cast<LogicT*>(instance);
}

template <typename LogicT>
inline void enterRealtimeAdapter(void* instance) {
  static_cast<LogicT*>(instance)->enterRealtime();
}

template <typename LogicT>
inline void exitRealtimeAdapter(void* instance) {
  static_cast<LogicT*>(instance)->exitRealtime();
}

template <typename LogicT>
inline void runCycleAdapter(void* instance) {
  static_cast<LogicT*>(instance)->run();
}

template <typename LogicT>
inline const ecmcCppLogicItemBinding* getItemBindingsAdapter(void* instance) {
  return static_cast<LogicT*>(instance)->ecmc.data();
}

template <typename LogicT>
inline uint32_t getItemBindingCountAdapter(void* instance) {
  return static_cast<LogicT*>(instance)->ecmc.count();
}

template <typename LogicT>
inline const ecmcCppLogicExportedVar* getExportedVarsAdapter(void* instance) {
  return static_cast<LogicT*>(instance)->epics.data();
}

template <typename LogicT>
inline uint32_t getExportedVarCountAdapter(void* instance) {
  return static_cast<LogicT*>(instance)->epics.count();
}

}  // namespace detail

}  // namespace ecmcCpp

#define ECMC_CPP_LOGIC_REGISTER(LOGIC_TYPE, LOGIC_NAME)                             \
  extern "C" const ecmcCppLogicApi* ecmc_cpp_logic_get_api(void) {                  \
    static_assert(std::is_base_of_v<ecmcCpp::LogicBase, LOGIC_TYPE>,                \
                  #LOGIC_TYPE " must derive from ecmcCpp::LogicBase");              \
    static const ecmcCppLogicApi kApi = {                                           \
      ECMC_CPP_LOGIC_ABI_VERSION,                                                    \
      LOGIC_NAME,                                                                    \
      &ecmcCpp::detail::setHostServicesAdapter<LOGIC_TYPE>,                          \
      &ecmcCpp::detail::createInstanceAdapter<LOGIC_TYPE>,                           \
      &ecmcCpp::detail::enterRealtimeAdapter<LOGIC_TYPE>,                            \
      &ecmcCpp::detail::exitRealtimeAdapter<LOGIC_TYPE>,                             \
      &ecmcCpp::detail::destroyInstanceAdapter<LOGIC_TYPE>,                          \
      &ecmcCpp::detail::runCycleAdapter<LOGIC_TYPE>,                                 \
      &ecmcCpp::detail::getItemBindingsAdapter<LOGIC_TYPE>,                          \
      &ecmcCpp::detail::getItemBindingCountAdapter<LOGIC_TYPE>,                      \
      &ecmcCpp::detail::getExportedVarsAdapter<LOGIC_TYPE>,                          \
      &ecmcCpp::detail::getExportedVarCountAdapter<LOGIC_TYPE>,                      \
    };                                                                               \
    return &kApi;                                                                    \
  }

#define ECMC_CPP_LOGIC_REGISTER_DEFAULT(LOGIC_TYPE) \
  ECMC_CPP_LOGIC_REGISTER(LOGIC_TYPE, #LOGIC_TYPE)
