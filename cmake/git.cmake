set(OUT_DIR ${CMAKE_BINARY_DIR}/generated/git)
set(GEN_H ${OUT_DIR}/git.h)

add_custom_command(
    OUTPUT ${GEN_H}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMAND ${PROJECT_PYTHON}
            -m
            scripts.git
            --output-dir ${OUT_DIR}
    DEPENDS
        ${PROJECT_PYTHON_DEPS}
        ${CMAKE_SOURCE_DIR}/scripts/git.py
    COMMENT "Generating Git hash/branch header"
    USES_TERMINAL
)

add_custom_target(generate_git_header DEPENDS ${GEN_H})
add_library(git_generated INTERFACE)
add_dependencies(git_generated generate_git_header)
target_sources(git_generated INTERFACE ${GEN_H})
target_include_directories(git_generated INTERFACE ${CMAKE_BINARY_DIR}/generated)
