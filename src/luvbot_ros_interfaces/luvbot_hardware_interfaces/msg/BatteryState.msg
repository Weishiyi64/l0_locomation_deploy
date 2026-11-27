# 电池快照：物理量在 3588 解码。以 YB-BMS_CAN_V2.0_2026101301.dbc 为准。
# ROS 字段使用小写下划线；注释保留DBC 原始 SignalName、中文描述和单位。
# NaN 表示未收到/无效浮点值；整数未收到为 255，循环次数为 UINT32_MAX。
# header 为 ROS 接收时间；其他字段均对应电池上报信号。
# 已收到的慢速/过期数据仍保留最后解码值；本话题不输出链路诊断信息。
std_msgs/Header header

# 常用电池数据
# BMS_PackStatus_SOC | 非冻结 SOC | %（手册范围 0～120）
float32 soc
# BMS_PackStatus_SOH | 电池 SOH | %
float32 soh
# BMS_PackStatus_SOP | 电池 SOP | kW（DBC 单位 kw，原始值 × 0.1）
float32 sop
# BMS_Status_PortState | 0=空载，1=放电，2=充电，3=关闭；未收到=255
uint8 charge_state
# BMS_Status_IsFullCharge | 满充状态：false=未满充，true=满充
bool full_charge
# BMS_PackStatus_Voltage | 电池电压 | V
float32 voltage
# BMS_PackStatus_Current | 电池电流 | A；保留 DBC 原始方向
# current = uint16_le(Byte6..7) × 0.01 - 327.68；不根据 charge_state 翻转符号。
# 已观察到放电为负；零点 0x8000，实际充电读数仍可用仪表复核。
float32 current

# 0x210 BMS_Info
# BMS_ComMatrixVersion | BMS通信矩阵版本
uint8 bms_com_matrix_version
# BMS_DeviceInfo_HVMajor | 电池硬件主版本号
uint8 bms_device_info_hv_major
# BMS_DeviceInfo_HVMinor | 电池硬件次版本号
uint8 bms_device_info_hv_minor
# BMS_DeviceInfo_SVMajor | 电池软件主版本号
uint8 bms_device_info_sv_major
# BMS_DeviceInfo_SVMinor | 电池软件次版本号
uint8 bms_device_info_sv_minor

# 0x211 BMS_Status
# BMS_AliveCnt4Charger | 握手和心跳计数器
uint8 bms_alive_cnt4_charger
# BMS_Status_PortState → charge_state（见消息开头）
# BMS_Status_IsAwake | 唤醒状态
# 按手册原始位：false=awake，true=sleep；IsAwake 名称与位值含义相反。
bool bms_status_is_awake
# BMS_Status_IsBalancing | 均衡状态
bool bms_status_is_balancing
# BMS_Status_ChargeAllowed | 允许充电
bool bms_status_charge_allowed
# BMS_Status_DischargeAllowed | 允许放电
bool bms_status_discharge_allowed
# BMS_Status_DischargeMosState | 放电MOS状态
bool bms_status_discharge_mos_state
# BMS_Status_ChargeMosState | 充电MOS状态
bool bms_status_charge_mos_state
# BMS_Status_ProtectionState | 保护状态
bool bms_status_protection_state
# BMS_Status_WarningState | 警告状态
bool bms_status_warning_state
# DchMos2State | 放电MOS2状态 | Byte4
uint8 bms_status_discharge_mos2_state
# ChgMos2State | 充电MOS2状态 | Byte3
uint8 bms_status_charge_mos2_state
# DchMos3State | 放电MOS3状态 | Byte6
uint8 bms_status_discharge_mos3_state
# ChgMos3State | 充电MOS3状态 | Byte5
uint8 bms_status_charge_mos3_state
# PreMosState | 预充MOS状态
uint8 bms_status_pre_mos_state

# 0x214 BMS_PackInfo
# BMS_PackInfo_ChemicalType | 电芯化学类型
uint8 bms_pack_info_chemical_type
# BMS_PackInfo_PortType | 电池包端口类型：0=同口，1=异口
uint8 bms_pack_info_port_type
# BMS_PackInfo_CellParallelNum | 电芯并联数量
uint8 bms_pack_info_cell_parallel_num
# BMS_PackInfo_CellSeriesNum | 电芯串联数量
uint8 bms_pack_info_cell_series_num
# BMS_PackInfo_RatedVoltage | 电池包标称电压 | V
float32 bms_pack_info_rated_voltage

# 0x216 BMS_PackStatus2
# BMS_PackStatus_SOCDisplay | 显示SOC | %
float32 bms_pack_status_soc_display
# BMS_PackStatus_TempMax | 电池包最高温度 | ℃
float32 bms_pack_status_temp_max
# BMS_PackStatus_TempMin | 电池包最低温度 | ℃
float32 bms_pack_status_temp_min
# BMS_PackStatus_CellVoltDiff | 电芯最大压差 | mV
float32 bms_pack_status_cell_volt_diff
# BMS_PackStatus_CellVoltMax | 电芯最高电压 | mV
float32 bms_pack_status_cell_volt_max
# BMS_PackStatus_CellVoltMin | 电芯最低电压 | mV
float32 bms_pack_status_cell_volt_min

# 0x217 BMS_Capacity
# BMS_Capacity_Design | 电池包设计容量 | wh
float32 bms_capacity_design
# BMS_Capacity_FullCharge | 电池包满充容量 | wh
float32 bms_capacity_full_charge
# BMS_Capacity_Current | 电池包当前容量 | wh
float32 bms_capacity_current
# BMS_Capacity_CycleCount | 电池包循环次数 | 次
uint32 bms_capacity_cycle_count

# 0x218 BMS_ChargeInfo
# BMS_ChargeVoltageReq | 充电请求电压 | V
float32 bms_charge_voltage_req
# BMS_ChargeCrrentReq | 充电请求电流 | A
# Crrent 为手册原始拼写，保留以便查找。
float32 bms_charge_crrent_req
# BMS_ChargedCapacity | 单次已充电量 | Ah
float32 bms_charged_capacity

# BMS_CellVoltNum | 单体电压数量
uint8 bms_cell_volt_num

# 0x219 BMS_CellVoltage
# BCU_Vol_1 | 单体1电压 | V
float32 bcu_vol_1
# BCU_Vol_2 | 单体2电压 | V
float32 bcu_vol_2
# BCU_Vol_3 | 单体3电压 | V
float32 bcu_vol_3
# BCU_Vol_4 | 单体4电压 | V
float32 bcu_vol_4

# 0x21A BMS_CellVoltage2
# BCU_Vol_5 | 单体5电压 | V
float32 bcu_vol_5
# BCU_Vol_6 | 单体6电压 | V
float32 bcu_vol_6
# BCU_Vol_7 | 单体7电压 | V
float32 bcu_vol_7
# BCU_Vol_8 | 单体8电压 | V
float32 bcu_vol_8

# 0x21B BMS_CellVoltage3
# BCU_Vol_9 | 单体9电压 | V
float32 bcu_vol_9
# BCU_Vol_10 | 单体10电压 | V
float32 bcu_vol_10
# BCU_Vol_11 | 单体11电压 | V
float32 bcu_vol_11

# SumVolt | 单体电压总和 | V
float32 sum_volt

# 0x21C BMS_ProtectionLast；DBC 未给出下列状态/故障值的枚举，保留原始整数。
# AswSocGetClbrtFlag | SOC 标定标志
uint8 asw_soc_get_clbrt_flag
# GetBmsFaultLevel | 故障等级
uint8 get_bms_fault_level
# GetSysState | 系统状态
uint8 get_sys_state
# GetStep | 步骤
uint8 get_step
# GetFaultList | 故障列表字段（8位整数，不扩展为未知故障码数组）
uint8 get_fault_list
# GetFaultNum | 故障数量字段
uint8 get_fault_num
# realsoc | 真实 SOC | %，原始值 × 0.1
float32 realsoc

# 0x21D BMS_Button
# KEY_WAKEUP | key唤醒
# 唤醒字段保留原始位，不锁存事件。
bool key_wakeup
# CHG_DET_WAKEUP | 充电唤醒
bool chg_det_wakeup
# IO_DET__WAKEUP | IO唤醒
bool io_det_wakeup

# ExVolt2 | V
float32 ex_volt2
# ChgVolt | V
float32 chg_volt
# InterVolt | V
float32 inter_volt
# GetSysKeyIO | 原始位
bool get_sys_key_io
# GetChgWakeUp | 原始位
bool get_chg_wake_up

# 0x21E BMS_Temperature；DBC 只有5路，Byte0～4，范围 -50～205 ℃。
# BMS_Temp_Cell1 | 电芯温度值1 | ℃
float32 bms_temp_cell1
# BMS_Temp_Cell2 | 电芯温度值2 | ℃
float32 bms_temp_cell2
# BMS_Temp_MOS1 | MOS温度1 | ℃
float32 bms_temp_mos1
# BMS_Temp_MOS2 | MOS温度2 | ℃
float32 bms_temp_mos2
# BMS_Temp_PSDG | 预放电阻温度 | ℃
float32 bms_temp_psdg
