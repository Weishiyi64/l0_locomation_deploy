/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * 帧分块适配器: 把上游任意粒度的帧流切成"板端单次 RPC 最大帧数"的固定大小 TCP 段。
 * 板端 eRPC/RPMsg 的 codec 固定编码 FRAME_BATCH_MAX 个槽位(40), 不足的批次
 * 会背空载荷跑一趟 RPC; 本适配器保证每段恰好凑满, 把 RPC 次数压到 帧率/40。
 */

#ifndef HPM_TRANSPORT_FRAME_CHUNKER_HPP
#define HPM_TRANSPORT_FRAME_CHUNKER_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "hpm_transport/frame.hpp"

#ifndef HPM_RPC_FRAME_BATCH
#define HPM_RPC_FRAME_BATCH 40U
#endif

namespace frameTransport {

class FrameChunker
{
public:
    explicit FrameChunker(size_t max_frames = HPM_RPC_FRAME_BATCH);

    void push(const uint8_t *wire);             /* 输入一帧 (kFrameWireSize 字节) */
    bool pop_segment(uint8_t *out, size_t *nframes); /* 有满段则取出并返回 true */
    size_t pending() const;                     /* 缓冲中未发出的帧数 */
    void flush(uint8_t *out, size_t *nframes);  /* 收尾: 取出剩余不足一段的帧 */

private:
    const size_t max_frames_;
    std::vector<uint8_t> buf_;
};

}  // namespace frameTransport

#endif /* HPM_TRANSPORT_FRAME_CHUNKER_HPP */