zephyr_library_amend()
set_source_files_properties(
    ${ZEPHYR_BASE}/drivers/display/display_st730x.c
    PROPERTIES HEADER_FILE_ONLY ON
)

zephyr_library_sources_ifdef(CONFIG_ST730X		display_st730x.c)