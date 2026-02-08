#include "ti_msp_dl_config.h"
#include "app_main.h"

int main(void)
{
    SYSCFG_DL_init();
    app_main();

    return 0;
}
