// Copyright 2023 Stefan Zellmann and Jefferson Amstutz
// SPDX-License-Identifier: Apache-2.0

#pragma once

// std
#include <ostream>
// anari-math
#include <anari/anari_cpp/ext/linalg.h>

inline std::ostream &operator<<(std::ostream &out, const anari::math::float3 &v)
{
  out << '(' << v.x << ',' << v.y << ',' << v.z << ')';
  return out;
}
inline std::ostream &operator<<(std::ostream &out, const anari::math::float4 &v)
{
  out << '(' << v.x << ',' << v.y << ',' << v.z << ',' << v.w << ')';
  return out;
}
inline std::ostream &operator<<(std::ostream &out, const anari::math::mat3 &m)
{
  out << '(' << m.x << ',' << m.y << ',' << m.z << ')';
  return out;
}
inline std::ostream &operator<<(std::ostream &out, const anari::math::mat4 &m)
{
  out << '(' << m.x << ',' << m.y << ',' << m.z << ',' << m.w << ')';
  return out;
}

inline anari::math::mat4 frustum(
    float left, float right, float bottom, float top, float znear, float zfar)
{
  anari::math::mat4 M;

  M[0][0] = (2.f * znear) / (right - left);
  M[1][0] = 0.f;
  M[2][0] = (right + left) / (right - left);
  M[3][0] = 0.f;

  M[0][1] = 0.f;
  M[1][1] = (2.f * znear) / (top - bottom);
  M[2][1] = (top + bottom) / (top - bottom);
  M[3][1] = 0.f;

  M[0][2] = 0.f;
  M[1][2] = 0.f;
  M[2][2] = -(zfar + znear) / (zfar - znear);
  M[3][2] = -(2.f * zfar * znear) / (zfar - znear);

  M[0][3] = 0.f;
  M[1][3] = 0.f;
  M[2][3] = -1.f;
  M[3][3] = 0.f;

  return M;
}
