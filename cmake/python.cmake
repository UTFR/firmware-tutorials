find_package(Python3 REQUIRED)

set(REQUIREMENTS_FILE ${CMAKE_SOURCE_DIR}/scripts/requirements.txt)

execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import cantools, yaml"
    RESULT_VARIABLE PYTHON_DEPS_MISSING
    OUTPUT_QUIET
    ERROR_QUIET
)

if(PYTHON_DEPS_MISSING EQUAL 0)
    set(PROJECT_PYTHON ${Python3_EXECUTABLE})
    set(PROJECT_PYTHON_DEPS "")
else()
    message(STATUS "System python3 is missing required packages - provisioning a venv in scripts/.venv")
    set(VENV_DIR ${CMAKE_SOURCE_DIR}/scripts/.venv)
    if(WIN32)
        set(VENV_PYTHON ${VENV_DIR}/Scripts/python.exe)
    else()
        set(VENV_PYTHON ${VENV_DIR}/bin/python)
    endif()

    add_custom_command(
        OUTPUT ${VENV_PYTHON}
        COMMAND ${Python3_EXECUTABLE} -m venv ${VENV_DIR}
        COMMAND ${VENV_PYTHON} -m pip install -r ${REQUIREMENTS_FILE}
        COMMENT "Creating Python virtual environment"
        USES_TERMINAL
    )
    add_custom_target(python_venv DEPENDS ${VENV_PYTHON})

    set(PROJECT_PYTHON ${VENV_PYTHON})
    set(PROJECT_PYTHON_DEPS ${VENV_PYTHON})
endif()
