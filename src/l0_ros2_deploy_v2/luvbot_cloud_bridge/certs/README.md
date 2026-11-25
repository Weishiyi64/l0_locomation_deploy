# 本机设备证书

将 `<device_id>.crt` 和 `<device_id>.key` 放在本目录，当前设备为 `LUV2026090001`。
这些文件保存在 ROS 程序目录内，被 Git 忽略；同步到机器人时请单独复制并保持私钥权限为 600。

执行 `colcon build --packages-select luvbot_cloud_bridge` 时，证书会安装到本包的
`share/luvbot_cloud_bridge/certs/`，节点默认通过 ament 索引查找。不要使用 sudo 构建。
更换证书后重新构建、重启节点，或显式使用 `cert_dir` 指定此目录。
