# 单个锁状态消息
uint8 board_address    # 板地址 (1-32)
uint8 lock_address     # 锁地址 (1-24)
bool is_locked         # 锁状态: true=已锁, false=已开
