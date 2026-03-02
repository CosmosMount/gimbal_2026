# wbr_gimbal
STM32H723 code for wheel-legged balanced robot gimbal

## 键位
| SW_L   | RobotMode |
| ------ | --------- |
| MIDDLE | MOVE |
| OTHERS | ALL_RELAX |

| SW_R   | FireMode |
| ------ | --------- |
| MIDDLE | FIRE |
| OTHERS | ALL_RELAX |

| MOUSE        | RobotMode       | Triggering Condition |
| ------------ | --------------- | -------------------- |
| MouseX       | pitch           | Push-Pull |
| MouseY       | yaw             | Push-Pull |

| KBD  | Command | Triggering Condition |
| ------ | --------- | -------------------- |
| W | 前进 | Press On |
| S | 后退 | Press On |
| A | 左走 | Press On |
| D | 右走 | Press On |
| R | 飞坡模式 | Press On |
| Q | 一键换头 | Click Start |
| E | 切换目标 | Click Start |
| CTRL+B | 重置UI | Press On |

| KBD/MOUSE | RobotMode      | Triggering Condition |
| --------- | -------------- | -------------------- |
| V         | 单发模式切换 | Click Start |
| Z         | 慢速拨弹盘 | Click Start |
| X         | 快速拨弹盘 | Click Start |
| C         | 快速拨弹盘 | Click Start |
| MouseL    | 手动击打 | Click Start |
| MouseR    | 自瞄 | Click Start |

| KBD  | RobotMode| Triggering Condition |
| ------ | ---------| -------------------- |
| CTRL+Q | 腿长-- | Press On |
| CTRL+E | 腿长++ | Press On |
| F      | 跳跃准备 | Press On |
| G      | 跳跃开始 | Press On |