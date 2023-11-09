// Copyright 2023 Stefan Zellmann and Jefferson Amstutz
// SPDX-License-Identifier: Apache-2.0

#pragma once

// anari-math
#include <anari/anari_cpp/ext/linalg.h>
// helpers on top of anari-math
#include "math-helpers.h"
using namespace anari::math;

static float3 unprojectNDC(mat4 projInv, mat4 viewInv, float3 ndc)
{
  anari::math::mul(projInv, float4(ndc, 1.f));
  float4 v =
      anari::math::mul(viewInv, anari::math::mul(projInv, float4(ndc, 1.f)));
  return float3(v.x, v.y, v.z) / v.w;
}

static bool intersectPlanePlane(
    float3 na, float3 pa, float3 nb, float3 pb, float3 &nl, float3 &pl)
{
  float3 nc = cross(na, nb);
  float det = length2(nc);

  if (det == 0.f)
    return false;

  float da = -dot(na, pa);
  float db = -dot(nb, pb);

  pl = ((cross(nc, nb) * da) + (cross(na, nc) * db)) / det;
  nl = nc;
  return true;
}

static bool solve(mat3 A, float3 b, float3 &x)
{
  float D = determinant(A);
  float D1 = determinant(mat3(b, A[1], A[2]));
  float D2 = determinant(mat3(A[0], b, A[2]));
  float D3 = determinant(mat3(A[0], A[1], b));

  if (D == 0.f)
    return false;

  x.x = D1 / D;
  x.y = D2 / D;
  x.z = D3 / D;
  return true;
}

static void closestLineSegmentBetweenTwoLines(
    float3 na, float3 pa, float3 nb, float3 pb, float3 &pc1, float3 &pc2)
{
  float3 nc = normalize(cross(na, nb));
  float3 b = pb - pa;
  mat3 A(na, -nb, nc);
  float3 x;
  if (!solve(A, b, x))
    return;
  pc1 = pa + na * x.x;
  pc2 = pb + nb * x.y;
}

static void offaxisStereoTransform(
    float3 LL, float3 LR, float3 UR, float3 eye, mat4 &projOUT, mat4 &viewOUT)
{
  float3 X = (LR - LL) / length(LR - LL);
  float3 Y = (UR - LR) / length(UR - LR);
  float3 Z = cross(X, Y);

  // from world to eye coords (eye at origin, looking down -Z)
  mat3 R = inverse(mat3(X, Y, Z));

  // eye position relative to screen/wall
  float3 eyeP = eye - LL;

  // distance from eye to screen/wall
  float dist = dot(eyeP, Z);

  float left = dot(eyeP, X);
  float right = length(LR - LL) - left;
  float bottom = dot(eyeP, Y);
  float top = length(UR - LR) - bottom;

  float znear = 1e-3f, zfar = 1000.f; // not relevant to us here

  left = -left * znear / dist;
  right = right * znear / dist;
  bottom = -bottom * znear / dist;
  top = top * znear / dist;

  projOUT = frustum(left, right, bottom, top, znear, zfar);
  viewOUT =
      mat4(float4(X, 0.f), float4(Y, 0.f), float4(Z, 0.f), float4(-eye, 1.f));
}

static void offaxisStereoCamera(float3 LL,
    float3 LR,
    float3 UR,
    float3 eye,
    float3 &dirOUT,
    float3 &upOUT,
    float &fovyOUT,
    float &aspectOUT,
    float4 &imageRegionOUT)
{
  float3 X = (LR - LL) / length(LR - LL);
  float3 Y = (UR - LR) / length(UR - LR);
  float3 Z = cross(X, Y);

  dirOUT = -Z;
  upOUT = Y;

  // eye position relative to screen/wall
  float3 eyeP = eye - LL;

  // distance from eye to screen/wall
  float dist = dot(eyeP, Z);

  float left = dot(eyeP, X);
  float right = length(LR - LL) - left;
  float bottom = dot(eyeP, Y);
  float top = length(UR - LR) - bottom;

  float newWidth = left < right ? 2 * right : 2 * left;
  float newHeight = bottom < top ? 2 * top : 2 * bottom;

  fovyOUT = 2 * atan(newHeight / (2 * dist));

  aspectOUT = newWidth / newHeight;

  imageRegionOUT.x = left < right ? (right - left) / newWidth : 0.f;
  imageRegionOUT.y = bottom < top ? (top - bottom) / newHeight : 0.f;
  imageRegionOUT.z = right < left ? (left + right) / newWidth : 1.f;
  imageRegionOUT.w = top < bottom ? (bottom + top) / newHeight : 1.f;
}

static void offaxisStereoCameraFromTransform(mat4 projInv,
    mat4 viewInv,
    float3 &eyeOUT,
    float3 &dirOUT,
    float3 &upOUT,
    float &fovyOUT,
    float &aspectOUT,
    float4 &imageRegionOUT)
{
  // Transform NDC unit cube corners to world/CAVE space
  float3 v000 = unprojectNDC(projInv, viewInv, float3(-1, -1, -1));
  float3 v001 = unprojectNDC(projInv, viewInv, float3(-1, -1, 1));

  float3 v100 = unprojectNDC(projInv, viewInv, float3(1, -1, -1));
  float3 v101 = unprojectNDC(projInv, viewInv, float3(1, -1, 1));

  float3 v110 = unprojectNDC(projInv, viewInv, float3(1, 1, -1));
  float3 v111 = unprojectNDC(projInv, viewInv, float3(1, 1, 1));

  float3 v010 = unprojectNDC(projInv, viewInv, float3(-1, 1, -1));
  float3 v011 = unprojectNDC(projInv, viewInv, float3(-1, 1, 1));

  // edges from -z to +z
  float3 ez00 = normalize(v001 - v000);
  float3 ez10 = normalize(v101 - v100);
  float3 ez11 = normalize(v111 - v110);
  float3 ez01 = normalize(v011 - v010);

  // edges from -y to +y
  float3 ey00 = normalize(v010 - v000);
  float3 ey10 = normalize(v110 - v100);
  float3 ey11 = normalize(v111 - v101);
  float3 ey01 = normalize(v011 - v001);

  // edges from -x to +x
  float3 ex00 = normalize(v100 - v000);
  float3 ex10 = normalize(v110 - v010);
  float3 ex11 = normalize(v111 - v011);
  float3 ex01 = normalize(v101 - v001);

  float3 nL = normalize(cross(ey00, ez00));
  float3 nR = normalize(cross(ez10, ey10));
  float3 nB = normalize(cross(ez00, ex00));
  float3 nT = normalize(cross(ex10, ez01));

  // Line of intersection between left/right planes
  float3 pLR, nLR;
  bool isectLR = intersectPlanePlane(nL, v000, nR, v100, nLR, pLR);

  // Line of intersection between bottom/top planes
  float3 pBT, nBT;
  bool isectBT = intersectPlanePlane(nB, v000, nT, v010, nBT, pBT);

  // Line segment connecting the two intersecint lines
  float3 p1, p2;
  closestLineSegmentBetweenTwoLines(nLR, pLR, nBT, pBT, p1, p2);

  eyeOUT = (p1 + p2) / 2.f;

  float3 LL = unprojectNDC(projInv, viewInv, float3(-1, -1, 1));
  float3 LR = unprojectNDC(projInv, viewInv, float3(1, -1, 1));
  float3 UR = unprojectNDC(projInv, viewInv, float3(1, 1, 1));

  offaxisStereoCamera(
      LL, LR, UR, eyeOUT, dirOUT, upOUT, fovyOUT, aspectOUT, imageRegionOUT);
}
