// Copyright 2024 NVIDIA Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosidl_buffer_registry/buffer_backend_registry.hpp"

#include <iostream>
#include <mutex>
#include <stdexcept>

#include <pluginlib/class_loader.hpp>

namespace rosidl_buffer_registry
{

BufferBackendRegistry::BufferBackendRegistry()
: plugins_loaded_(false)
{
  try {
    loader_ = std::make_unique<pluginlib::ClassLoader<BufferBackend>>(
      "rosidl_buffer_registry",
      "rosidl_buffer_registry::BufferBackend");
  } catch (const std::exception & e) {
    // Silently ignore if pluginlib isn't available - will work without plugins
    loader_ = nullptr;
  }
}

BufferBackendRegistry::~BufferBackendRegistry()
{
  // CRITICAL: Clear all global state before ClassLoader is destroyed
  clear_global_state();
}

BufferBackendRegistry & BufferBackendRegistry::get_instance()
{
  static BufferBackendRegistry instance;
  return instance;
}

std::shared_ptr<BufferBackend> BufferBackendRegistry::get_backend(const std::string & name)
{
  // Ensure plugins are loaded on first access
  if (!plugins_loaded_) {
    load_plugins();
  }

  auto it = backends_.find(name);
  if (it != backends_.end()) {
    return it->second;
  }

  return nullptr;
}

void BufferBackendRegistry::register_backend(
  const std::string & name,
  std::shared_ptr<BufferBackend> backend)
{
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  backends_[name] = backend;
}

void BufferBackendRegistry::load_plugins()
{
  static std::once_flag load_flag;
  std::call_once(
    load_flag, [this]() {
      if (!loader_) {
        plugins_loaded_ = true;
        return;
      }

      try {
        auto declared_classes = loader_->getDeclaredClasses();

        std::cerr << "[BufferBackendRegistry] Discovered " << declared_classes.size() <<
          " plugin(s)\n";

        if (declared_classes.empty()) {
          std::cerr << "[BufferBackendRegistry] No buffer backend plugins found\n";
        } else {
          for (const auto & class_name : declared_classes) {
            std::cerr << "[BufferBackendRegistry] Loading plugin: " << class_name << "\n";
            try {
              auto backend = loader_->createSharedInstance(class_name);
              // Use the class name as the backend name (e.g., "cuda")
              register_backend(class_name, backend);
              std::cerr << "[BufferBackendRegistry] Successfully registered: " << class_name <<
                "\n";
            } catch (const std::exception & e) {
              std::cerr << "[BufferBackendRegistry] Failed to load " << class_name << ": " <<
                e.what() << "\n";
              continue;
            }
          }
        }
      } catch (const std::exception & e) {
        std::cerr << "[BufferBackendRegistry] Plugin discovery error: " << e.what() << "\n";
      }

      plugins_loaded_ = true;
    });
}

std::vector<std::string> BufferBackendRegistry::get_backend_names() const
{
  std::vector<std::string> names;
  names.reserve(backends_.size());
  for (const auto & pair : backends_) {
    names.push_back(pair.first);
  }
  return names;
}

void BufferBackendRegistry::clear_global_state()
{
  // CRITICAL: Clear backends_ map to release shared_ptr to plugin instances
  // before ClassLoader is destroyed.
  backends_.clear();

  // Note: Global serialization maps in rosidl_typesupport_fastrtps_cpp are cleared
  // by rmw_shutdown() which is called before this destructor runs.
}

}  // namespace rosidl_buffer_registry
