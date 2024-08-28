// Copyright 2023 Simon Hoffmann
#pragma once
#include <stdexcept>
namespace tam::helpers
{
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept
{
  return static_cast<typename std::underlying_type<E>::type>(e);
}
}  // namespace tam::helpers
