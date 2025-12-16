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

#ifndef ROSIDL_RUNTIME_CPP__BUFFER_IMPL_BASE_HPP_
#define ROSIDL_RUNTIME_CPP__BUFFER_IMPL_BASE_HPP_

#include <cstddef>
#include <memory>
#include <string>

namespace rosidl_runtime_cpp
{

/// Abstract base class for buffer backend implementations.
/// Provides minimal interface for metadata and conversion operations.
template<typename T>
class BufferImplBase
{
public:
  virtual ~BufferImplBase() = default;

  /// Get the number of elements in the buffer.
  virtual size_t size() const = 0;

  /// Resize the buffer to contain n elements.
  virtual void resize(size_t n) = 0;

  /// Clear the buffer, removing all elements.
  virtual void clear() = 0;

  /// Get opaque backend-specific handle (e.g., device pointer).
  /// Returns nullptr for empty buffers.
  virtual const void * get_backend_handle() const = 0;

  /// Convert buffer contents to CPU memory.
  /// Creates a new CpuBufferImpl containing a copy of the data.
  /// @return A unique_ptr to a CPU buffer implementation.
  virtual std::unique_ptr<BufferImplBase<T>> to_cpu() const = 0;

  // ========== Descriptor-based Serialization Interface ==========

  /// Get the fully qualified ROS message type name for the descriptor.
  /// Examples: "cpu_buffer_msgs/msg/CpuBufferDescriptor",
  ///           "cuda_buffer_msgs/msg/CudaBufferDescriptor"
  /// @return Message type name used for serialization.
  virtual std::string get_descriptor_type_name() const = 0;

  /// Create a descriptor message for serialization.
  /// The descriptor contains backend-specific metadata needed for transmission.
  /// @return Type-erased shared_ptr to the descriptor message.
  /// @note The actual type matches get_descriptor_type_name().
  virtual std::shared_ptr<void> create_descriptor() const = 0;

  /// Reconstruct buffer implementation from a descriptor message.
  /// Used during deserialization to recreate the buffer from received metadata.
  /// @param descriptor Type-erased shared_ptr to descriptor message.
  /// @return New buffer implementation reconstructed from descriptor.
  virtual std::unique_ptr<BufferImplBase<T>> from_descriptor(
    const std::shared_ptr<void> & descriptor) const = 0;

  /// Create a deep copy of this buffer implementation.
  /// Used to implement value semantics for Buffer<T>.
  /// @return New buffer implementation containing a copy of this buffer's data.
  virtual std::unique_ptr<BufferImplBase<T>> clone() const = 0;
};

}  // namespace rosidl_runtime_cpp

#endif  // ROSIDL_RUNTIME_CPP__BUFFER_IMPL_BASE_HPP_

