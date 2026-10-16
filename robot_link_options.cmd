/* 主循环含浮点姿态计算，预留 2 KB 栈；与生成的 device_linker.cmd 一起链接。 */
--stack_size=2048
-uinterruptVectors
