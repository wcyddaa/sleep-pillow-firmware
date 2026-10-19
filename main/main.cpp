#include "app_main.h"
#include "board.h"

extern "C" void app_main() {
    // 显式引用板型实现，确保 libboard.a 中的具体板型被链接进固件
    BoardEnsureRegistered();

    AppMain app;
    app.Run();
}
