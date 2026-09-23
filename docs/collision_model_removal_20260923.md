# 移除桌面与额外夹爪保护盒（2026-09-23）

根据本次用户要求，新旧抓取流程不再加载桌面碰撞模型或额外夹爪保护盒，不依赖桌面点云建模、保存文件、模型有效期或桌面场景验收。
固定放置参考姿态、20 cm 下降、释放与退回顺序保留。此次没有连接真实机械臂或发送真实运动。

## 当前行为

- `run_robot_prepare.sh` 和其现有备用入口 `run_robot_prepare_o.sh` 在 MoveIt 就绪后执行旧对象清理，RViz 的 Planning Scene 随 MoveIt 场景更新。
- 抓取后端在每次规划前也清理 `grasp_table` 和 `grasp_gripper_envelope`，避免复用已经启动的 MoveIt 时留下旧对象。
- 清理范围仅为上述两个对象，包括世界对象、附着对象及它们在允许碰撞矩阵中的条目。保留其他对象、机器人 URDF 几何、SRDF 自碰撞规则；不把避碰参数全局关闭。
- MoveIt 的附着对象 REMOVE 会先把物体放回世界，因此同一 diff 同时删除其世界对象，随后回读确认。不仅取消附着或隐藏 RViz 显示。
- 不再读取或创建桌面模型，也不再要求环境场景非空。关节限位、IK、轨迹、速度、TF、External Control 和控制器检查保留。
- 视觉配置 `table_roi.example.json` 只是图像识别区域，仍保留；它不会建立桌面碰撞模型。

## 文件变更

| 文件 | 修改 |
| --- | --- |
| `src/grasp_executor/grasp_executor/config.py` | 移除全部桌面配置和必须存在环境模型的开关 |
| `src/grasp_executor/config/config.json`、`place_config.json` | 新旧流程均删除桌面字段 |
| `src/grasp_executor/grasp_executor/moveit_ros.py` | 移除模型加载、哈希/有效期/底座锚点校验、保护盒添加及执行前桌面校验；规划前执行定向清理 |
| `src/grasp_executor/grasp_executor/scene_cleanup.py` | 新增旧场景对象清理和回读验证，无建模功能 |
| `tools/clear_legacy_grasp_scene.py` | 准备脚本使用的清理入口，不发送机械臂/夹爪命令 |
| `run_robot_prepare.sh`、`run_robot_prepare_o.sh` | MoveIt 就绪后清理旧场景 |
| `run_grasp_executor.sh`、`run_pipeline_place_execute.sh` | 更新说明，取消桌面建模前提 |
| `src/grasp_executor/test/test_scene_cleanup.py` | 对象迁移、矩阵保留、重复清理、失败回读、两套配置测试 |
| `tools/test_moveit_grasp.py` | 无桌面模型规划、旧场景迁移、三段/六段模拟执行测试 |
| `New_run/README.md`、启动/适配文档 | 删除建模启动步骤，历史实施文档标注已停用 |

已删除：`run_table_model.sh`、`New_run/run_table_model.sh`、`tools/build_table_model.py`、`tools/test_table_model.py`、`grasp_executor/table_model.py`、`grasp_executor/table_scene.py`。

修改前代码备份：`debug_output/change_backups/before_remove_table_20260923.tar.gz`。
旧 `~/.local/state/robot_grasp_ws/table_model.json` 已移至 `debug_output/change_backups/table_model_before_removal_20260923.json`，不再位于原加载位置。

## 启动顺序

第一终端：

```bash
cd /home/rob/robot_grasp_ws/New_run
./run_robot_prepare.sh
```

第二终端先做新抓取放置流程的预览：

```bash
cd /home/rob/robot_grasp_ws/New_run
./run_pipeline_place_test.sh --vision-config ../src/yolo_vision/config/table_roi.example.json
```

预览通过后，真实执行入口仍为 `./run_pipeline_place_execute.sh`，可使用同一 `--vision-config` 参数。
旧的只抓取不放置流程仍使用 `run_pipeline_test.sh` / `run_pipeline_execute.sh`。
不再运行建模脚本；pipeline 自动启动抓取、定位和 YOLO 节点。

## 测试结果和边界

1. `colcon build --symlink-install --packages-select yolo_vision rim_locator grasp_executor`：三个包通过。
2. `python3 -m pytest -q src/grasp_executor/test src/rim_locator/test src/yolo_vision/test tools/test_pipeline_runner.py tools/test_robot_control.py tools/test_joint_stationarity.py`：80 passed、7 skipped。跳过项是默认未启用的 `ROBOT_STARTUP_ROS_TEST` 模拟启动图测试，不是测试失败。
3. `tools/test_moveit_grasp.py`：隔离 DDS 域、localhost，真实 MoveIt 服务配合模拟关节状态和模拟轨迹 Action。通过以下九项：准备入口清理；后端清理；三段规划；固定参考六段规划；三段模拟执行；六段模拟执行及释放退回；工具 TF/FK；不可达拒绝；独立障碍碰撞拒绝。最终控制台记录：`debug_output/collision_removal_20260923/moveit_final.log`；MoveIt 节点日志：`debug_output/collision_removal_20260923/moveit_final/`。
4. 安装空间验证：两个旧 Python 模块均不可导入；新清理模块和新旧配置能从构建后的环境加载。
5. 启动脚本逐文件 `bash -n`、`git diff --check` 通过。

修复过的测试失败：首次清理只解除附着，导致保护盒变成世界对象；已补上世界对象 REMOVE 并通过真实 MoveIt 回读验证。另外，测试从默认参数切换到生产配置后，IK 求解预算由 5 秒变成 15 秒，原测试客户端 15 秒截止时间会竞争超时；测试现按实际求解预算额外留 5 秒响应余量，生产流程的 120 秒规划预算未变。

这些结果验证代码和模拟接口，不代表现场目标一定有 IK 解，也不代表机械臂已完成真实抓取。当前版本不会检测未建模的桌面；机器人 URDF 自碰撞检查保持开启。真实相机采集、机械臂运动和夹持效果尚未在本次验证。
