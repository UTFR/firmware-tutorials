# Libraries declare a recipe once; boards instantiate what they name.
# Declaration must happen before any board is added.
#
# Two boards with different chips cannot share a compiled library, so per-board
# instantiation stays -- this replaces the boilerplate around it, not the
# instantiation itself.

# utfr_declare_lib(<name>
#     [TYPE OBJECT|STATIC]              default OBJECT
#     [SOURCES ...]                     relative to the declaring directory
#     [INCLUDES ...] [PRIVATE_INCLUDES ...]
#     [DEPS ...]                        unsuffixed targets (UTFR_UTILS, ...)
#     [BOARD_DEPS ...]                  suffixed with -<board> at instantiation
#     [PRIVATE_BOARD_DEPS ...]
#     [DEFINES ...]                     @BOARD@ / @KIND@ expand at instantiation
# )
function(utfr_declare_lib name)
    cmake_parse_arguments(L "" "TYPE"
        "SOURCES;INCLUDES;PRIVATE_INCLUDES;DEPS;BOARD_DEPS;PRIVATE_BOARD_DEPS;DEFINES"
        ${ARGN})

    if(NOT L_TYPE)
        set(L_TYPE OBJECT)
    endif()

    foreach(p DIR TYPE SOURCES INCLUDES PRIVATE_INCLUDES DEPS BOARD_DEPS PRIVATE_BOARD_DEPS DEFINES)
        set(v "${L_${p}}")
        if(p STREQUAL DIR)
            set(v "${CMAKE_CURRENT_LIST_DIR}")
        endif()
        set_property(GLOBAL PROPERTY UTFR_LIB_${name}_${p} "${v}")
    endforeach()

    set_property(GLOBAL APPEND PROPERTY UTFR_DECLARED_LIBS ${name})
endfunction()

function(utfr_instantiate_lib name board)
    get_property(declared GLOBAL PROPERTY UTFR_DECLARED_LIBS)
    if(NOT name IN_LIST declared)
        message(FATAL_ERROR "Board ${board} wants '${name}', which no library declares.")
    endif()
    if(TARGET ${name}-${board})
        return()
    endif()

    foreach(p DIR TYPE SOURCES INCLUDES PRIVATE_INCLUDES DEPS BOARD_DEPS PRIVATE_BOARD_DEPS DEFINES)
        get_property(${p} GLOBAL PROPERTY UTFR_LIB_${name}_${p})
    endforeach()

    set(target ${name}-${board})
    add_library(${target} ${TYPE})

    foreach(s ${SOURCES})
        target_sources(${target} PRIVATE ${DIR}/${s})
    endforeach()

    if(INCLUDES)
        target_include_directories(${target} PUBLIC ${INCLUDES})
    endif()
    if(PRIVATE_INCLUDES)
        target_include_directories(${target} PRIVATE ${PRIVATE_INCLUDES})
    endif()

    set(public_deps ${DEPS})
    foreach(d ${BOARD_DEPS})
        list(APPEND public_deps ${d}-${board})
    endforeach()
    if(public_deps)
        target_link_libraries(${target} ${public_deps})
    endif()

    set(private_deps "")
    foreach(d ${PRIVATE_BOARD_DEPS})
        list(APPEND private_deps ${d}-${board})
    endforeach()
    if(private_deps)
        target_link_libraries(${target} PRIVATE ${private_deps})
    endif()

    foreach(def ${DEFINES})
        string(REPLACE "@BOARD@" "${board}" def "${def}")
        string(REPLACE "@KIND@" "${BOARD_KIND-${board}}" def "${def}")
        target_compile_definitions(${target} PRIVATE ${def})
    endforeach()
endfunction()

# Build everything `board` declared: its HAL, its RTOS, and its libraries.
function(utfr_add_board board)
    compile_hal(${board})

    if(BOARD_RTOS-${board})
        compile_freertos(${board})
    endif()

    foreach(lib ${BOARD_LIBS-${board}})
        utfr_instantiate_lib(${lib} ${board})
    endforeach()
endfunction()

