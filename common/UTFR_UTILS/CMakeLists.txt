add_library(UTFR_UTILS STATIC)
target_sources(UTFR_UTILS PRIVATE
    types.c
    utils.c
    bench_stat.c
)
target_include_directories(UTFR_UTILS PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(UTFR_UTILS )
