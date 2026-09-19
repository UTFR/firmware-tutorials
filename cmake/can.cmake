set(GENERATED_CAN_C "")
set(GENERATED_CAN_H "")
set(OUT_DIR ${CMAKE_BINARY_DIR}/generated/can_types)
set(DBC_DIR ${CMAKE_SOURCE_DIR}/dbc)

foreach(DBC ${CAN_DBC_FILES})
    get_filename_component(DBC_NAME ${DBC} NAME_WE)
    string(TOLOWER ${DBC_NAME} DB_NAME)
    list(APPEND GENERATED_CAN_C ${OUT_DIR}/${DB_NAME}.c)
    list(APPEND GENERATED_CAN_H ${OUT_DIR}/${DB_NAME}.h)
endforeach()

add_custom_command(
    OUTPUT ${GENERATED_CAN_C} ${GENERATED_CAN_H}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMAND ${PROJECT_PYTHON}
            -m scripts.dbc.can_autogen
            --yaml-dir   ${DBC_DIR}
            --output-dir ${OUT_DIR}
            --dbc-output-dir ${DBC_DIR}
    DEPENDS
        ${PROJECT_PYTHON_DEPS}
        ${CMAKE_SOURCE_DIR}/scripts/dbc/can_autogen.py
        ${CMAKE_SOURCE_DIR}/scripts/dbc/fmt.py
        ${CMAKE_SOURCE_DIR}/scripts/dbc/message.py
        ${CMAKE_SOURCE_DIR}/scripts/dbc/util.py
        ${CAN_YAML_FILES}
    COMMENT "Generating CAN types"
    USES_TERMINAL
)

add_library(can_generated STATIC ${GENERATED_CAN_C})

add_custom_target(can_types_gen
  DEPENDS ${GENERATED_CAN_H}
)

add_library(can_generated_headers INTERFACE)
add_dependencies(can_generated_headers can_types_gen)
target_include_directories(can_generated_headers INTERFACE
  ${CMAKE_BINARY_DIR}/generated
)
