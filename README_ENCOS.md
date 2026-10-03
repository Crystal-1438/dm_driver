# ENCOS（原 MT4）CAN 驱动

本驱动按用户提供的 **《ENCOS 电机调试技术手册 V1.20EAP》**修正。保留 `MT4_motor`、`MT4motor_*` 名称，实际协议是 ENCOS；不是达妙 MIT 协议。

资料：[Notion 中的 ENCOS 手册](https://app.notion.com/p/ENCOS-V1-20EAP-608e3df668544cf8b19dbb11d7641313)。本仓库不附带手册和临时附件下载凭据。电机型号未知，因此量程必须由调用者明确配置，不能把手册某个型号的参数当作所有电机的默认值。

## 本次修正

| 原行为 | 当前行为及手册依据 |
|---|---|
| KD 先量化 12 位再截断/饱和为 9 位 | 直接按 `0..511` 量化；物理上下限来自配置（§9.1.1、表 9-1） |
| 把反馈的 12 位字段当扭矩 | 解析为相电流 A，使用独立电流量程；只有配置 Kt 后才计算估算扭矩（§10.1） |
| byte6 当 MOS 温度，丢弃 byte7 | byte6 电机温度、byte7 MOS 温度；`(raw-50)/2.0`，保留半度（§10.1） |
| 不区分帧类型和长度 | 按 byte0 高 3 位及各类型 DLC 解析，拒绝畸形帧（§10） |
| 回调忽略 CAN 总线编号 | 按总线编号和电机 ID 共同匹配 |
| 位置自动按 25 rad 展开并折回 2π | 删除这些 SDK 特有推断，保留协议原始位置及显式坐标偏置 |
| 周期插入“使能”阻尼帧 | 删除；没有手册支持的专用使能帧 |
| “失能”发大 KD | 软件 STOP 默认发精确零电流 `61 00 00`；可配置变阻尼制动 `69 00 00`（§9.1.4） |
| 存零点只写 ID 低字节 | 完整写入 ID 高/低字节，处理成功/失败应答（§7.2） |
| 未定义错误码 | 解析 0..7 错误，其余保留为未知（§10） |

这些是通信与主控软件行为的修正，尚未连接真实电机验证。

## 文件与构建

- `encos_protocol.h/.c`：纯 C 编解码，无 HAL 依赖；支持混控、伺服位置、伺服速度、电流/力矩、制动、零点设置、设置 ID、查询 ID 和常用参数查询的组帧。
- `MT4_motor.h/.c`：接入现有 BSP 的周期混控驱动、停止、查询、在线检查及反馈更新。
- `tests/`：主机测试，使用手册十六进制报文和模拟 CAN/时钟，不发送真实 CAN。

在 STM32 工程中加入 `encos_protocol.c`、`MT4_motor.c`、现有 controller/PID、monitor 和帧率统计模块。建议 C11。仍需你的平台提供：

- `board_def.h`：`_HAL_MT4MOTOR_ENABLE=1`、`DEVICE_CAN_CNT`。
- `bsp_can.h`：`BSP_CAN_Send`、`BSP_CAN_RegisterRxCallback`、`BSP_CAN_AddFilter`、`basic_data_t`。回调首参数必须是 CAN 控制器编号；标准数据帧、RTR/扩展帧过滤由 BSP 负责；DLC 须转换为实际字节数。发送接口须在返回前复制数据，不能异步保留栈上指针。
- `bsp_time.h`：`BSP_sys_time_ms()`，单调毫秒计数，允许 uint32 回绕。
- `common.h`/`cvector.h`：`RT_MALLOC`、配对的 **`RT_FREE`**、vector、`_MID`、`fsgn` 等现有依赖。若底层用 `rt_malloc`，释放必须配 `rt_free`，不要混用 libc 堆。
- `Monitor_Loop()` 每 1 ms 调用一次；`MT4motor_Is_Online` 和运动反馈新鲜度另用真实毫秒计时。

同一对象的 RX 更新、控制循环、参数修改和 Enable/Disable 必须串行执行。建议 CAN ISR 只入队，在控制任务中解析；驱动不提供 RTOS 锁，也不保证多个字段在 ISR/任务并发访问时形成一致快照。

## 配置与初始化

旧 `max_position/max_speed/max_torque/max_kp/max_kd` 改为 `config.ranges`，还新增独立电流量程。所有六个范围必须有限、`min < max`，KP/KD 下限须为零，位置/速度/电流/扭矩范围须包含零。未填写或非法配置，`MT4motor_Create` 返回 NULL。

以下示例数值仅对应手册 §9.1.1 所举 **EC-A4310-P2-36**；不代表用户当前 X5 电机。接线前先通过手册、厂商或查询确认实际范围。量程是协议映射，不是安全运行限值；主控还应按实际机构限制角度、速度和力矩。

```c
MT4_motor_config cfg = {0};
cfg.bsp_can_index = 0;
cfg.can_tx_id = cfg.can_rx_id = 1;  // 根据实际 CAN ID 设置
cfg.ranges = (ENCOS_Ranges){
    .kp       = {0.0f, 500.0f},
    .kd       = {0.0f, 5.0f},
    .position = {-12.5f, 12.5f},  // rad
    .speed    = {-18.0f, 18.0f},  // rad/s
    .torque   = {-30.0f, 30.0f},  // Nm，指令前馈扭矩
    .current  = {-30.0f, 30.0f},  // A，反馈相电流
};
cfg.torque_constant = 0.0f;     // 未知：fdb_torque=NAN，torque_valid=0
cfg.position_offset = 0.0f;     // rad
cfg.feedback_timeout_ms = 100;  // 主控策略，可配置
cfg.stop_mode = mt4_soft_stop;  // 零电流；不是硬件失能
cfg.fre_rel = 1;                // 每次 Calc_Send 都发送

MT4_motor *motor = MT4motor_Create(&cfg, my_control_callback);
if (motor == NULL) { /* 配置错误或内存不足，禁止进入控制 */ }
```

`my_control_callback(MT4_motor *m)` 由应用定义，更新 `m->kp/kd/ref_position/ref_speed/ref_torque`。传 NULL 则运行已有主控 PID，PID 输出按 Nm 写入前馈扭矩。主控 controller 的位置、速度反馈仍沿用度、度/秒；MT4 的 `ref_position/fdb_position` 为 rad，`ref_speed/fdb_speed` 为 rad/s。单位必须区分。

`position_offset` 的定义：`real_fdb_position = fdb_position + offset`，发送目标时减去 offset。没有自动跨圈、自动存零或首次上电跟随当前位置功能。启用前由应用设置目标。

### 运行与停止

1. 初始化 BSP/monitor，创建电机；初始状态为 `mt4_stop`，创建本身不发送报文。
2. 周期调用 `MT4motor_Calc_Send()`。停止态发送零电流/配置的制动帧，请求类型 1 反馈，可持续读取位置。
3. 获得新鲜的位置、速度反馈后，设置目标，再调用 `MT4motor_Enable()`，检查返回值。
4. 正常控制时每个发送周期只有一帧混控，没有额外的 300 ms “使能帧”。
5. `MT4motor_Disable()` 立即置软件停止并发送停止帧。反馈超时、已报告错误或非有限数值指令也会使驱动停止；恢复通信不会自动恢复运动，需重新启用。

`Enable` 只改变主控软件状态。`Disable`/SOFT 只发零电流，不表示电源或功率级物理失能，也不会主动释放机械抱闸。需要阻尼制动时设置 `mt4_damping_stop`；机械抱闸另有协议，驱动不会自动操作。

`feedback_timeout_ms` 是主控策略，不修改电机内部 CAN 超时设置。手册提到电机默认 500 ms、也可关闭，实际必须查询确认。主控软件停摆时无法发送零电流，因此不能把主控超时检测当作电机端看门狗。

### 读取反馈与参数

- `fdb_current`：相电流 A。
- `fdb_torque`：Kt 已知时的估算 Nm，可能受非线性 Kt 补偿影响；不是直接测量值。
- `t_rotor/t_mos`：℃。类型 2/3 没有 MOS 温度，旧值不会被伪造刷新。
- `feedback.valid`：当前这帧包含哪些物理量；`feedback_valid`：历史接收过哪些量。
- `last_position_ms/last_speed_ms`：各自更新时间。配置/查询应答不能冒充新鲜运动反馈。
- `update`：接受新报文置 1，调用者处理后清零。
- `MT4motor_Is_Online()`：最近是否收到已支持且合法的反馈；不等于电机无错误或允许运动。

查询示例：`MT4motor_Query(motor, 24)` 查询 KD 范围。第二参数是十进制代码，不要把 24 当作 `0x24`：

| 十进制代码 | 含义 | 解码结果 |
|---|---|---|
| 1 / 2 / 3 / 4 | 位置(度)/速度(RPM)/电流(A)/功率 | `feedback.query_value`；前三项也更新对应标准单位字段 |
| 5 | 加速度 rad/s² | `feedback.query_value` |
| 22 | Kt Nm/A | `feedback.query_value` |
| 23 / 24 | KP / KD 范围 | `feedback.query_range.min/max` |
| 25 / 26 | 位置(rad)/速度(rad/s)范围 | 同上 |
| 27 / 28 | 扭矩(Nm)/电流(A)范围 | 同上 |
| 31 | 电机内部 CAN 超时 ms | `feedback.query_value` |

发送返回 `ENCOS_OK` 仅表示提交到 BSP，不表示收到应答。等待 `feedback.type == 5` 且 `feedback.code` 匹配；应用需要自行处理超时、重试及接收数据快照。驱动不会自动将查询范围应用到配置，更不会写回电机。

型号完全未知、尚无法创建电机对象时，可先使用纯编解码层的 `ENCOS_PackQuery`/`ENCOS_Decode`，通过 BSP 只发送查询指令。类型 5 范围应答不依赖量程，`ENCOS_Decode(data,len,NULL,&reply)` 即可解析。读取并核对后再填配置和创建电机；不要为进入查询流程猜测量程。

### 零点设置

`MT4motor_Save_Zero()` 发 `0x7FF: ID_H ID_L 00 03`。调用前停住电机，确保同总线所有本驱动电机均已处于 STOP；函数不自动让运动中的电机归零。

- 发送后，该 CAN 总线中本驱动的所有发送暂停 **501 ms**，其他 CAN 总线不受影响。
- 应用必须同时暂停同总线其他发送者（例如 DM 驱动或直接 BSP 发送），否则无法满足手册 §3.5 的总线指令间隔要求。
- `zero_pending` 表示等待应答；`zero_ack_received/zero_ack_success` 区分成功、失败和未收到。
- ACK 等待策略为 1000 ms，需持续调用 Calc_Send 才推进超时处理。失败/超时不要假定零点设置成功；核对反馈后由应用决定是否重试。
- 新的控制必须等待新的位置、速度反馈并显式重新启用。

## 明确保留的限制

- 未确认硬件型号、固件与实际量程；未进行真实电机或 STM32 整机编译测试。
- `MT4motor_Clear_Err` 返回 `ENCOS_UNSUPPORTED`，手册未定义通用清错指令。不会发送达妙 `FF...FB`。
- `MT4motor_Reset_Round` 返回 `ENCOS_UNSUPPORTED`；旧 round/wrap 字段和推断算法已移除。
- `MT4motor_Broadcast_Cmd` 只兼容子命令 3（无偏置存零）。设置 ID/查询 ID 的纯组帧函数不自动发送、不自动修改过滤器或路由；ID 更新流程必须在停机维护时由应用管理。
- 不实现带角度偏置的存零、无寻址的 ID 重置、参数写入、分段 Kt 查询代码 36、EtherCAT/CAN FD 适配。
- 解析类型 1/2/3/4/5/6；类型 5 支持代码 1..5、22..35、37..39。其他类型/长度、配置章节中的 `FF FE...` 特殊回包返回 unsupported/invalid，不刷新在线时间，也不会被当作运动反馈。
- 不自动更改电机参数或机械抱闸。没有完整发送队列/任务同步，BSP 的实际发送错误不在当前返回值内（原 BSP 接口没有错误返回约定）。
- 手册 §11 的混控文字示例与 §9.1.1 的量化范围存在不一致。本实现采用 §9.1.1 的字段定义和明确的十六进制示例，不按 §11 的文字直接截取物理整数。查询首字节用模式 7、保留位为零的 `E0`；§9.3 示例 `E1` 的低五位标为无效保留位。

## 无硬件验证

```bash
bash tests/run_tests.sh
SANITIZE=1 bash tests/run_tests.sh
```

覆盖官方黄金报文、KD 5/50 上限、负数、温度半度、反馈类型/长度、非有限数值、跨总线同 ID、启动停止、掉线锁止、零点等待和 32 位时钟回绕。测试代码不会访问硬件。

在受 ptrace 管理的环境中 LeakSanitizer 无法启动时，可用：

```bash
ASAN_OPTIONS=detect_leaks=0 SANITIZE=1 bash tests/run_tests.sh
```

此时仍运行 AddressSanitizer 和 UndefinedBehaviorSanitizer，但不验证泄漏。上板顺序：核对实际量程和固件 → 查询参数 → 零电流反馈监测 → 核对单位/方向/温度 → 低输出测试控制 → 检查拔线/断主控时的电机端保护。
