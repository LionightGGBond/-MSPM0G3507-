/* CCS会自动编译SDK官方启动文件，本文件在CCS构建时不再定义向量表。
 * 独立命令行验证没有CCS的自动依赖，显式定义下面的宏后引用官方实现。 */
#if defined(ROBOT_STANDALONE_STARTUP)
#include <ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c>
#else
typedef int RobotStartupProvidedByCCS;
#endif
