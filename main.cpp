// Copyright 2023 Stefan Zellmann and Jefferson Amstutz
// SPDX-License-Identifier: Apache-2.0

// anari_cpp
#define ANARI_EXTENSION_UTILITY_IMPL
#include <anari/anari_cpp.hpp>
// std
#include <algorithm>
#include <array>
#include <cstdio>
#include <iostream>
#include <numeric>
#include <random>
// stb_image
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
// ours
#include "math-helpers.h"

#include "Projection.h"

// ========================================================
// generate our test scene
// ========================================================
anari::World generateScene(anari::Device device, const float3 &pos)
{
  const uint32_t numSpheres = 10000;
  const float radius = .015f;

  std::mt19937 rng;
  rng.seed(0);
  std::normal_distribution<float> vert_dist(0.f, 0.25f);

  // Create + fill position and color arrays with randomized values //

  auto indicesArray = anari::newArray1D(device, ANARI_UINT32, numSpheres);
  auto positionsArray =
      anari::newArray1D(device, ANARI_FLOAT32_VEC3, numSpheres);
  auto distanceArray = anari::newArray1D(device, ANARI_FLOAT32, numSpheres);
  {
    auto *positions = anari::map<float3>(device, positionsArray);
    auto *distances = anari::map<float>(device, distanceArray);
    for (uint32_t i = 0; i < numSpheres; i++) {
      const auto a = positions[i][0] = vert_dist(rng);
      const auto b = positions[i][1] = vert_dist(rng);
      const auto c = positions[i][2] = vert_dist(rng);
      distances[i] = std::sqrt(a * a + b * b + c * c); // will be roughly 0-1
      // translate
      positions[i] += pos;
    }
    anari::unmap(device, positionsArray);
    anari::unmap(device, distanceArray);

    auto *indicesBegin = anari::map<uint32_t>(device, indicesArray);
    auto *indicesEnd = indicesBegin + numSpheres;
    std::iota(indicesBegin, indicesEnd, 0);
    std::shuffle(indicesBegin, indicesEnd, rng);
    anari::unmap(device, indicesArray);
  }

  // Create and parameterize geometry //

  auto geometry = anari::newObject<anari::Geometry>(device, "sphere");
  anari::setAndReleaseParameter(
      device, geometry, "primitive.index", indicesArray);
  anari::setAndReleaseParameter(
      device, geometry, "vertex.position", positionsArray);
  anari::setAndReleaseParameter(
      device, geometry, "vertex.attribute0", distanceArray);
  anari::setParameter(device, geometry, "radius", radius);
  anari::commitParameters(device, geometry);

  // Create color map texture //

  auto texelArray = anari::newArray1D(device, ANARI_FLOAT32_VEC3, 2);
  {
    auto *texels = anari::map<float3>(device, texelArray);
    texels[0][0] = 1.f;
    texels[0][1] = 0.f;
    texels[0][2] = 0.f;
    texels[1][0] = 0.f;
    texels[1][1] = 1.f;
    texels[1][2] = 0.f;
    anari::unmap(device, texelArray);
  }

  auto texture = anari::newObject<anari::Sampler>(device, "image1D");
  anari::setAndReleaseParameter(device, texture, "image", texelArray);
  anari::setParameter(device, texture, "filter", "linear");
  anari::commitParameters(device, texture);

  // Create and parameterize material //

  auto material = anari::newObject<anari::Material>(device, "matte");
  anari::setAndReleaseParameter(device, material, "color", texture);
  anari::commitParameters(device, material);

  // Create and parameterize surface //

  auto surface = anari::newObject<anari::Surface>(device);
  anari::setAndReleaseParameter(device, surface, "geometry", geometry);
  anari::setAndReleaseParameter(device, surface, "material", material);
  anari::commitParameters(device, surface);

  // Create and parameterize world //

  auto world = anari::newObject<anari::World>(device);
#if 1
  {
    auto surfaceArray = anari::newArray1D(device, ANARI_SURFACE, 1);
    auto *s = anari::map<anari::Surface>(device, surfaceArray);
    s[0] = surface;
    anari::unmap(device, surfaceArray);
    anari::setAndReleaseParameter(device, world, "surface", surfaceArray);
  }
#else
  anari::setAndReleaseParameter(
      device, world, "surface", anari::newArray1D(device, &surface));
#endif
  anari::release(device, surface);
  anari::commitParameters(device, world);

  return world;
}

// ========================================================
// query anari extensions (ANARI_VSNRAY_CAMERA_MATRIX)
// ========================================================
static bool deviceHasExtension(anari::Library library,
    const std::string &deviceSubtype,
    const std::string &extName)
{
  const char **extensions =
      anariGetDeviceExtensions(library, deviceSubtype.c_str());

  for (; *extensions; extensions++) {
    if (*extensions == extName)
      return true;
  }
  return false;
}

// ========================================================
// Log ANARI errors
// ========================================================
static void statusFunc(const void * /*userData*/,
    ANARIDevice /*device*/,
    ANARIObject source,
    ANARIDataType /*sourceType*/,
    ANARIStatusSeverity severity,
    ANARIStatusCode /*code*/,
    const char *message)
{
  if (severity == ANARI_SEVERITY_FATAL_ERROR) {
    fprintf(stderr, "[FATAL][%p] %s\n", source, message);
    std::exit(1);
  } else if (severity == ANARI_SEVERITY_ERROR) {
    fprintf(stderr, "[ERROR][%p] %s\n", source, message);
  } else if (severity == ANARI_SEVERITY_WARNING) {
    fprintf(stderr, "[WARN ][%p] %s\n", source, message);
  } else if (severity == ANARI_SEVERITY_PERFORMANCE_WARNING) {
    fprintf(stderr, "[PERF ][%p] %s\n", source, message);
  }
  // Ignore INFO/DEBUG messages
}

// ========================================================
// Function to render a given frame (renderer+world+cam)
//  and produce an output image
// ========================================================
static void render(
    anari::Device device, anari::Frame frame, const std::string &fileName)
{
  // Render frame and print out duration property //

  anari::render(device, frame);
  anari::wait(device, frame);

  float duration = 0.f;
  anari::getProperty(device, frame, "duration", duration, ANARI_NO_WAIT);

  printf("rendered frame in %fms\n", duration * 1000);

  stbi_flip_vertically_on_write(1);
  auto fb = anari::map<uint32_t>(device, frame, "channel.color");
  stbi_write_png(
      fileName.c_str(), fb.width, fb.height, 4, fb.data, 4 * fb.width);
  anari::unmap(device, frame, "channel.color");

  std::cout << "Output: " << fileName << '\n';
}

// ========================================================
// Strategy 1
//  requires an ANARI extension, provided by the
//  anari-visionaray device
// ========================================================
static void renderMatricesWithMatrixCamExtension(
    anari::Device device, anari::Frame frame, mat4 proj, mat4 view)
{
  // Create camera //

  auto camera = anari::newObject<anari::Camera>(device, "matrix");

  anari::setParameter(device, camera, "proj", proj);
  anari::setParameter(device, camera, "view", view);

  anari::commitParameters(device, camera);

  anari::setParameter(device, frame, "camera", camera);
  anari::commitParameters(device, frame);

  render(device, frame, "strategy1.png");

  anari::release(device, camera);
}

// ========================================================
// Strategy 2
// ========================================================
static void renderFixedFrameWithPerspectiveCam(anari::Device device,
    anari::Frame frame,
    float3 LL,
    float3 LR,
    float3 UR,
    float3 eye)
{
  float3 dir, up;
  float fovy, aspect;
  float4 imgRegion;
  offaxisStereoCamera(LL, LR, UR, eye, dir, up, fovy, aspect, imgRegion);

  // Create camera //

  auto camera = anari::newObject<anari::Camera>(device, "perspective");

  anari::setParameter(device, camera, "position", eye);
  anari::setParameter(device, camera, "direction", dir);
  anari::setParameter(device, camera, "up", up);
  anari::setParameter(device, camera, "fovy", fovy);
  anari::setParameter(device, camera, "aspect", aspect);
  anari::setParameter(
      device, camera, "imageRegion", ANARI_FLOAT32_BOX2, &imgRegion);

  anari::commitParameters(device, camera);

  anari::setParameter(device, frame, "camera", camera);
  anari::commitParameters(device, frame);

  render(device, frame, "strategy2.png");

  anari::release(device, camera);
}

// ========================================================
// Strategy 3
// ========================================================
static void renderMatricesWithPerspectiveCam(
    anari::Device device, anari::Frame frame, mat4 proj, mat4 view)
{
  float3 eye, dir, up;
  float fovy, aspect;
  float4 imgRegion;
  offaxisStereoCameraFromTransform(
      inverse(proj), inverse(view), eye, dir, up, fovy, aspect, imgRegion);

  // Create camera //

  auto camera = anari::newObject<anari::Camera>(device, "perspective");

  anari::setParameter(device, camera, "position", eye);
  anari::setParameter(device, camera, "direction", dir);
  anari::setParameter(device, camera, "up", up);
  anari::setParameter(device, camera, "fovy", fovy);
  anari::setParameter(device, camera, "aspect", aspect);
  anari::setParameter(
      device, camera, "imageRegion", ANARI_FLOAT32_BOX2, &imgRegion);

  anari::commitParameters(device, camera);

  anari::setParameter(device, frame, "camera", camera);
  anari::commitParameters(device, frame);

  render(device, frame, "strategy3.png");

  anari::release(device, camera);
}

int main()
{
  // Setup ANARI device //

  auto library = anari::loadLibrary("environment", statusFunc);
  auto device = anari::newDevice(library, "default");

  anari::Extensions extensions =
      anari::extension::getInstanceExtensionStruct(device, device);

  if (!extensions.ANARI_KHR_GEOMETRY_SPHERE)
    printf("WARNING: device doesn't support ANARI_KHR_GEOMETRY_SPHERE\n");
  if (!extensions.ANARI_KHR_CAMERA_PERSPECTIVE)
    printf("WARNING: device doesn't support ANARI_KHR_CAMERA_PERSPECTIVE\n");
  if (!extensions.ANARI_KHR_LIGHT_DIRECTIONAL)
    printf("WARNING: device doesn't support ANARI_KHR_LIGHT_DIRECTIONAL\n");
  if (!extensions.ANARI_KHR_MATERIAL_MATTE)
    printf("WARNING: device doesn't support ANARI_KHR_MATERIAL_MATTE\n");

  // Create world from a helper function //

  auto world = generateScene(device, float3(1.5f, 1.5f, 0.f));

  // Add a directional light source //

  auto light = anari::newObject<anari::Light>(device, "directional");
  anari::setParameterArray1D(device, world, "light", &light, 1);
  anari::release(device, light);

  // Create renderer //

  auto renderer = anari::newObject<anari::Renderer>(device, "default");
  const float4 backgroundColor = {0.1f, 0.1f, 0.1f, 1.f};
  anari::setParameter(device, renderer, "background", backgroundColor);
  anari::setParameter(device, renderer, "pixelSamples", 32);
  anari::commitParameters(device, renderer);

  // Create frame (top-level object) //

  auto frame = anari::newObject<anari::Frame>(device);

  uint2 imageSize = {800, 800};
  anari::setParameter(device, frame, "size", imageSize);
  anari::setParameter(device, frame, "channel.color", ANARI_UFIXED8_RGBA_SRGB);

  anari::setParameter(device, frame, "world", world);
  anari::setParameter(device, frame, "renderer", renderer);

  bool hasMatrixCameraExt =
      deviceHasExtension(library, "default", "ANARI_VSNRAY_CAMERA_MATRIX");

  // Input configuration: screen of size 3x3, viewer at the center
  // but with an offset in Y, so the frustum is tilted a little
  // towards the top
  float3 LL(0.f, 0.f, 0.f);
  float3 LR(3.f, 0.f, 0.f);
  float3 UR(3.f, 3.f, 0.f);
  float3 eye(1.5f, 1.68f, 1.5f);

  // Strategy 1: use matrices coming from the app, plus an extension that
  // unprojects rays in NDC back to world space
  // (the renderer has to support/implement this)
  if (hasMatrixCameraExt) {
    std::cout << "Strategy 1 ...\n";
    mat4 proj, view;
    offaxisStereoTransform(LL, LR, UR, eye, proj, view);
    renderMatricesWithMatrixCamExtension(device, frame, proj, view);
  } else {
    std::cerr
        << "Extension ANARI_VSNRAY_CAMERA_MATRIX not found, skipping Strategy 1\n";
  }

  // Strategy 2: transform the input frame to a format any ANARI device supports
  {
    std::cout << "Strategy 2 ...\n";
    renderFixedFrameWithPerspectiveCam(device, frame, LL, LR, UR, eye);
  }

  // Strategy 3: given the input matrices, first reconstruct the frustum,
  // then transform input frame as in Strategy 2
  {
    std::cout << "Strategy 3 ...\n";
    mat4 proj, view;
    offaxisStereoTransform(LL, LR, UR, eye, proj, view);
    renderMatricesWithPerspectiveCam(device, frame, proj, view);
  }

  // Cleanup remaining ANARI objets //

  anari::release(device, renderer);
  anari::release(device, world);
  anari::release(device, frame);
  anari::release(device, device);

  anari::unloadLibrary(library);

  return 0;
}
