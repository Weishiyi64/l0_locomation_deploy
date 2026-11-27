/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_transport/frame_chunker.hpp"

#include <cstring>

namespace frameTransport {

FrameChunker::FrameChunker(size_t max_frames) : max_frames_(max_frames), buf_()
{
    buf_.reserve(2U * max_frames_ * kFrameWireSize);
}

void FrameChunker::push(const uint8_t *wire)
{
    buf_.insert(buf_.end(), wire, wire + kFrameWireSize);
}

bool FrameChunker::pop_segment(uint8_t *out, size_t *nframes)
{
    const size_t need = max_frames_ * kFrameWireSize;

    if (buf_.size() < need) {
        return false;
    }
    std::memcpy(out, buf_.data(), need);
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(need));
    *nframes = max_frames_;
    return true;
}

size_t FrameChunker::pending() const
{
    return buf_.size() / kFrameWireSize;
}

void FrameChunker::flush(uint8_t *out, size_t *nframes)
{
    const size_t bytes = buf_.size();

    *nframes = 0U;
    if (bytes == 0U) {
        return;
    }
    std::memcpy(out, buf_.data(), bytes);
    buf_.clear();
    *nframes = bytes / kFrameWireSize;
}

}  // namespace frameTransport