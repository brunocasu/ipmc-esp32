idf_component_register(SRCS "src/openipmc_esp32_example_main.c"
                            "src/esp32_ipmc_ios.c"
                            "openipmc/src/ipmb_0.c"
                            "openipmc/src/ipmb_0.h"
                            "openipmc/src/ipmc_ios.h"
                            "openipmc/src/ipmc_tasks.h"
                    INCLUDE_DIRS "."
                    )
