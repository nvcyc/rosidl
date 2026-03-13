// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include "rosidl_buffer/buffer.hpp"
#include "rosidl_buffer/buffer_impl_base.hpp"
#include "rosidl_buffer/c_helpers.h"
#include "rosidl_buffer/cpu_buffer_impl.hpp"

using rosidl::Buffer;
using rosidl::BufferImplBase;
using rosidl::CpuBufferImpl;

/// Minimal non-CPU implementation for testing that C helpers throw on non-CPU buffers.
template<typename T>
class NonCpuBufferImpl : public BufferImplBase<T>
{
public:
  explicit NonCpuBufferImpl(size_t count)
  : size_(count) {}

  std::string get_backend_type() const override {return "non_cpu_test";}
  size_t size() const override {return size_;}

  std::unique_ptr<BufferImplBase<T>> to_cpu() const override
  {
    auto cpu = std::make_unique<CpuBufferImpl<T>>();
    cpu->get_storage().resize(size_);
    return cpu;
  }

  std::unique_ptr<BufferImplBase<T>> clone() const override
  {
    return std::make_unique<NonCpuBufferImpl<T>>(size_);
  }

private:
  size_t size_;
};

// -- rosidl_buffer_uint8_size --

TEST(TestCHelpers, size_empty_buffer) {
  Buffer<uint8_t> buf;
  EXPECT_EQ(0u, rosidl_buffer_uint8_size(&buf));
}

TEST(TestCHelpers, size_nonempty_buffer) {
  Buffer<uint8_t> buf(42);
  EXPECT_EQ(42u, rosidl_buffer_uint8_size(&buf));
}

TEST(TestCHelpers, size_after_push_back) {
  Buffer<uint8_t> buf;
  buf.push_back(1);
  buf.push_back(2);
  buf.push_back(3);
  EXPECT_EQ(3u, rosidl_buffer_uint8_size(&buf));
}

TEST(TestCHelpers, size_works_for_non_cpu_backend) {
  auto impl = std::make_unique<NonCpuBufferImpl<uint8_t>>(7);
  Buffer<uint8_t> buf(std::move(impl));
  EXPECT_EQ(7u, rosidl_buffer_uint8_size(&buf));
}

// -- rosidl_buffer_uint8_data (const) --

TEST(TestCHelpers, data_returns_pointer_to_contents) {
  Buffer<uint8_t> buf;
  buf.push_back(0xAA);
  buf.push_back(0xBB);
  buf.push_back(0xCC);

  const uint8_t * ptr = rosidl_buffer_uint8_data(&buf);
  ASSERT_NE(nullptr, ptr);
  EXPECT_EQ(0xAA, ptr[0]);
  EXPECT_EQ(0xBB, ptr[1]);
  EXPECT_EQ(0xCC, ptr[2]);
}

TEST(TestCHelpers, data_matches_buffer_data) {
  Buffer<uint8_t> buf(10, 0x42);
  EXPECT_EQ(buf.data(), rosidl_buffer_uint8_data(&buf));
}

TEST(TestCHelpers, data_throws_for_non_cpu_backend) {
  auto impl = std::make_unique<NonCpuBufferImpl<uint8_t>>(4);
  Buffer<uint8_t> buf(std::move(impl));
  EXPECT_THROW(rosidl_buffer_uint8_data(&buf), std::runtime_error);
}

// -- rosidl_buffer_uint8_data_mut --

TEST(TestCHelpers, data_mut_returns_mutable_pointer) {
  Buffer<uint8_t> buf(3, 0);
  uint8_t * ptr = rosidl_buffer_uint8_data_mut(&buf);
  ASSERT_NE(nullptr, ptr);

  ptr[0] = 10;
  ptr[1] = 20;
  ptr[2] = 30;

  EXPECT_EQ(10, buf[0]);
  EXPECT_EQ(20, buf[1]);
  EXPECT_EQ(30, buf[2]);
}

TEST(TestCHelpers, data_mut_matches_buffer_data) {
  Buffer<uint8_t> buf(5, 0xFF);
  EXPECT_EQ(buf.data(), rosidl_buffer_uint8_data_mut(&buf));
}

TEST(TestCHelpers, data_mut_throws_for_non_cpu_backend) {
  auto impl = std::make_unique<NonCpuBufferImpl<uint8_t>>(4);
  Buffer<uint8_t> buf(std::move(impl));
  EXPECT_THROW(rosidl_buffer_uint8_data_mut(&buf), std::runtime_error);
}

// -- rosidl_buffer_uint8_resize --

TEST(TestCHelpers, resize_grows_buffer) {
  Buffer<uint8_t> buf;
  EXPECT_EQ(0u, rosidl_buffer_uint8_size(&buf));

  rosidl_buffer_uint8_resize(&buf, 10);
  EXPECT_EQ(10u, rosidl_buffer_uint8_size(&buf));
  EXPECT_EQ(10u, buf.size());
}

TEST(TestCHelpers, resize_shrinks_buffer) {
  Buffer<uint8_t> buf(20, 0x55);
  rosidl_buffer_uint8_resize(&buf, 5);
  EXPECT_EQ(5u, buf.size());
}

TEST(TestCHelpers, resize_to_zero) {
  Buffer<uint8_t> buf(10, 1);
  rosidl_buffer_uint8_resize(&buf, 0);
  EXPECT_EQ(0u, buf.size());
  EXPECT_TRUE(buf.empty());
}

TEST(TestCHelpers, resize_preserves_existing_data) {
  Buffer<uint8_t> buf;
  buf.push_back(0xDE);
  buf.push_back(0xAD);

  rosidl_buffer_uint8_resize(&buf, 5);
  EXPECT_EQ(5u, buf.size());
  EXPECT_EQ(0xDE, buf[0]);
  EXPECT_EQ(0xAD, buf[1]);
}

TEST(TestCHelpers, resize_throws_for_non_cpu_backend) {
  auto impl = std::make_unique<NonCpuBufferImpl<uint8_t>>(4);
  Buffer<uint8_t> buf(std::move(impl));
  EXPECT_THROW(rosidl_buffer_uint8_resize(&buf, 10), std::runtime_error);
}

// -- rosidl_buffer_uint8_throw_if_not_cpu --

TEST(TestCHelpers, throw_if_not_cpu_does_not_throw_for_cpu) {
  Buffer<uint8_t> buf(5, 1);
  EXPECT_NO_THROW(rosidl_buffer_uint8_throw_if_not_cpu(&buf));
}

TEST(TestCHelpers, throw_if_not_cpu_does_not_throw_for_empty_cpu) {
  Buffer<uint8_t> buf;
  EXPECT_NO_THROW(rosidl_buffer_uint8_throw_if_not_cpu(&buf));
}

TEST(TestCHelpers, throw_if_not_cpu_throws_for_non_cpu_backend) {
  auto impl = std::make_unique<NonCpuBufferImpl<uint8_t>>(4);
  Buffer<uint8_t> buf(std::move(impl));
  EXPECT_THROW(rosidl_buffer_uint8_throw_if_not_cpu(&buf), std::runtime_error);
}

// -- Round-trip: write via data_mut, verify via data --

TEST(TestCHelpers, roundtrip_write_then_read) {
  Buffer<uint8_t> buf(4, 0);

  uint8_t * wptr = rosidl_buffer_uint8_data_mut(&buf);
  wptr[0] = 'R';
  wptr[1] = 'O';
  wptr[2] = 'S';
  wptr[3] = '2';

  const uint8_t * rptr = rosidl_buffer_uint8_data(&buf);
  EXPECT_EQ('R', rptr[0]);
  EXPECT_EQ('O', rptr[1]);
  EXPECT_EQ('S', rptr[2]);
  EXPECT_EQ('2', rptr[3]);
}

TEST(TestCHelpers, resize_then_fill_then_verify) {
  Buffer<uint8_t> buf;

  rosidl_buffer_uint8_resize(&buf, 256);
  EXPECT_EQ(256u, rosidl_buffer_uint8_size(&buf));

  uint8_t * wptr = rosidl_buffer_uint8_data_mut(&buf);
  for (size_t i = 0; i < 256; ++i) {
    wptr[i] = static_cast<uint8_t>(i);
  }

  const uint8_t * rptr = rosidl_buffer_uint8_data(&buf);
  for (size_t i = 0; i < 256; ++i) {
    EXPECT_EQ(static_cast<uint8_t>(i), rptr[i]);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
