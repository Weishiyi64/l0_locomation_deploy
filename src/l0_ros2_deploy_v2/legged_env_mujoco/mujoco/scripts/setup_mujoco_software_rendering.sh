#!/bin/bash

# 作用：启用软件渲染优化模式
# 效果：在代码中检测到此变量后，会：
# 禁用阴影（mjRND_SHADOW）
# 禁用反射（mjRND_REFLECTION）
# 禁用雾效（mjRND_FOG）
# 禁用大气（mjRND_HAZE）
# 禁用天空盒（mjRND_SKYBOX）
# 禁用叠加透明度（mjRND_ADDITIVE）
# 禁用线框模式（mjRND_WIREFRAME）
# 位置：在 glfw_adapter.cc 和 simulate.cc 中检测
export MUJOCO_SOFTWARE_RENDERING=0

# 作用：控制渲染帧跳过率
# 取值：
# 0：每帧都渲染
# 1：每 2 帧渲染一次
# 2：每 3 帧渲染一次
# 更高值：性能更好，流畅度更低
# 位置：在 simulate.cc 的 RenderLoop() 中读取
export MUJOCO_RENDER_SKIP=4

# 作用：强制 GLFW/OpenGL 使用软件渲染后端
# 效果：当系统无 GPU 驱动时，GLFW 会回退到 Mesa 软件渲染（llvmpipe）
# 适用：系统级设置，影响所有使用 GLFW 的程序
export LIBGL_ALWAYS_SOFTWARE=0

# 作用：默认不输出信息，避免干扰日志
# 启用输出：设置 MUJOCO_VERBOSE=1 后显示配置信息
export MUJOCO_VERBOSE=1

if [ "${MUJOCO_VERBOSE}" = "1" ]; then
    echo "=========================================="
    echo "MuJoCo Software Rendering Configuration"
    echo "=========================================="
    echo "Environment variables set:"
    echo "  MUJOCO_SOFTWARE_RENDERING=${MUJOCO_SOFTWARE_RENDERING} (1 disables expensive visual effects)"
    echo "  MUJOCO_RENDER_SKIP=${MUJOCO_RENDER_SKIP} (frame skipping rate)"
    echo "  LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE} (1 forces the Mesa software backend)"
    echo "=========================================="
fi
