# Draft: LSM6DSR 接口设计完善与规划

## 需求（已确认）
- 用户希望"完善与接口设计与高强度规划"
- **范围**: 针对 LSM6DSR 陀螺仪
- 目标: 完善驱动层/业务层接口设计，支持更好的平台隔离

## 当前架构状态
- **三层架构**: 驱动层(lsm6dsr) → 业务层(bsp_lsm6dsr) → 测试层(test_lsm6dsr)
- **I/O 抽象**: `lsm6dsr_io_t` 结构体（read/write 回调 + ctx 指针）
- **平台隔离**: 驱动层平台无关，测试层注入 STM32 HAL 实现

## 已知优势
- 驱动层可复用到其他 MCU
- 配置宏通过 `#ifndef` 定义，支持编译器覆盖
- 生产 API 清晰（init/calibrate/update/get_data/vofa_format）

## 待讨论
- [x] 范围: LSM6DSR 陀螺仪
- [ ] 驱动层需要哪些新功能？
- [ ] 业务层 API 是否需要扩展？
- [ ] 是否需要支持 SPI 接口？
- [ ] 测试策略（单元测试 vs 硬件测试）？

## 技术决策
（待记录）

## 范围边界
- INCLUDE: 待确认
- EXCLUDE: 待确认
