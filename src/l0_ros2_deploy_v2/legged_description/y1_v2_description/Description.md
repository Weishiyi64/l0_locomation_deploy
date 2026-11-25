## 🧸 机器人描述文件夹接口规范

机器人描述文件夹应作为子模块添加到项目中，用于描述机器人的物理结构和URDF/MJCF模型。

---

### 📋 命名规范

- **`{robot_type}`**: 机器人名称
  - **必须首字母小写**
  - 用于标识机器人类型

- **仓库名称**: `{robot_type}_description`
  - 格式：`{robot_type}` + `_description`

---

### 📁 必需文件

#### 1. MJCF 文件
- **文件名称**: `{robot_type}.xml`
- **存放位置**: `{robot_type}_description/mjcf/` 目录下
- **mujoco model参数**: `{robot_type}`

#### 2. URDF 文件
- **文件名称**: `{robot_type}.urdf`
- **存放位置**: `{robot_type}_description/urdf/` 目录下
- **robot name参数**: `{robot_type}`

---

### ⚙️ ROS相关配置

#### 1. URDF中mesh查找方式
- **路径格式**: 使用 `package://` 开始的路径查找方式
  - 将 `../meshes` 整体替换为 `package://{robot_type}/meshes`

#### 2. CMakeLists.txt
- **project参数**: `{robot_type}_description`

#### 3. package.xml
- **name参数**: `{robot_type}_description`