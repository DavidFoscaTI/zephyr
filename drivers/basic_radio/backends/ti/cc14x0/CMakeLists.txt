# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
    prop_rf_cc14xx_trx.c
    ti_cc14xx.c
    radio_settings/rcl_settings_wisun.c
    host/trx_host.c
    host/handlers/trx_host_handlers.c
    host/private/trx_host_private.c
    host/transport/spi_transport.c
    host/utility/utils.c
    host/utility/list/list.c
    host/utility/buffer_manager/buffer_manager.c
    host/hal/platform_util.c
    host/hal/platform_spi.c
)

# Backend-local headers (ti_cc14xx.h, host/*.h) — kept internal to this backend.
zephyr_library_include_directories(. host)
