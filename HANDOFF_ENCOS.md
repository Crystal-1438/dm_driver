# ENCOS 驱动接手记录

任务：根据用户 Notion 中《ENCOS 电机调试技术手册 V1.20EAP》修正 dm_driver 的 MT4（实为 ENCOS）实现。用户不知道电机型号，明确要求把量程放入 config。

先读 `README_ENCOS.md`，其中列出协议依据、完整迁移示例、BSP 契约和未支持部分。不要把旧 SDK 推断出的 KD=50、反馈扭矩、2π 回绕和阻尼使能逻辑重新加回。

## 关键边界

- 保留 MT4 名称；`MT4_motor_config` 改为显式 `ENCOS_Ranges ranges`，配置是来源于实机参数的 min/max，不是软件限速限矩。
- 未确定型号，不填隐式量程。允许直接用独立 codec 查询量程后再创建驱动对象。
- 力位混控反馈是相电流；Kt=0 时 `fdb_torque=NAN`。
- 软件 Disable 是零电流或显式选择的变阻尼制动，不是真正电机失能。
- 当前 BSP 是外部依赖，仓库不能单独生成 STM32 固件。主机测试通过，不代表电机实测通过。
- 未读到通用清错协议；Clear_Err 显式返回 unsupported。
- 测试 stub 仅用于主机，不能复制到固件替代真实 BSP。
- RT_MALLOC/RT_FREE 必须来自同一堆；生产 cvector 的分配失败契约需要平台提供，本库不能从 void 的 pushback 接口获得分配错误。
- RX/控制任务由调用者串行化；没有跨 ISR/任务锁。

## 后续必须从硬件确认

1. 完整型号、固件版本，查询 22..28 和 31，确认 Kt、六项量程及电机端看门狗。
2. 实测经典 CAN 1 Mbps、标准帧、反馈 DLC、温度位置和单位。
3. 仅零电流监测，核对协议位置是否发生实际环绕；手册未定义可供自动多圈展开的规则。
4. 使用真实 STM32 HAL/BSP 编译，确认 CAN RX 首参数是总线索引，FDCAN DLC 已转换为字节数，发送会复制栈上数据。
5. 如需伺服位置/速度/电流周期模式，在已有纯编码器基础上增设明确状态机；不要在现有混控周期中混发两种命令。
6. 如需参数写入或改 ID，新增确认/等待/路由变更流程和测试；不可仅发送后就假定配置成功。

运行测试：`bash tests/run_tests.sh`。内存/未定义行为检查：`ASAN_OPTIONS=detect_leaks=0 SANITIZE=1 bash tests/run_tests.sh`（当前受管环境 LeakSanitizer 受 ptrace 限制）。所有测试都在本机模拟 CAN，没有真实设备副作用。
