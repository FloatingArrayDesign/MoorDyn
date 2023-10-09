set(PYSRCS "${CMAKE_CURRENT_SOURCE_DIR}/moordyn/__init__.py"
           "${CMAKE_CURRENT_SOURCE_DIR}/moordyn/moordyn.py"
           "${CMAKE_CURRENT_SOURCE_DIR}/cmoordyn.cpp")

# Prepare the install script, injecting some information coming from cMake
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/pyproject.toml.in"
               "${CMAKE_CURRENT_BINARY_DIR}/tmp/pyproject.toml")
file(
    COPY "${CMAKE_CURRENT_BINARY_DIR}/tmp/pyproject.toml"
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
    FILE_PERMISSIONS OWNER_READ
                     OWNER_WRITE
                     OWNER_EXECUTE
                     GROUP_READ
                     GROUP_EXECUTE
                     WORLD_READ
                     WORLD_EXECUTE)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in"
               "${CMAKE_CURRENT_BINARY_DIR}/tmp/setup.py")
file(
    COPY "${CMAKE_CURRENT_BINARY_DIR}/tmp/setup.py"
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
    FILE_PERMISSIONS OWNER_READ
                     OWNER_WRITE
                     OWNER_EXECUTE
                     GROUP_READ
                     GROUP_EXECUTE
                     WORLD_READ
                     WORLD_EXECUTE)

# Create the target, depending of course on the main library
set(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/dist/moordyn.built")
add_custom_command(
    OUTPUT ${OUTPUT}
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/cmoordyn.cpp" "${CMAKE_CURRENT_BINARY_DIR}/cmoordyn.cpp"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/moordyn" "${CMAKE_CURRENT_BINARY_DIR}/moordyn"
    COMMAND ${Python_EXECUTABLE} -m build
    COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}
    DEPENDS ${PYSRCS}
)
add_custom_target(python_wrapper ALL DEPENDS ${OUTPUT})
add_dependencies(python_wrapper moordyn)

# Prepare the installation if eventually required
set(PY_INSTALL_ARGS "")
if(${PYTHON_WRAPPER_USERINSTALL})
    set(PY_INSTALL_ARGS "${PY_INSTALL_ARGS} --user")
endif()
if (UNIX)
    if((${Python_VERSION_MAJOR} GREATER 3) OR ((${Python_VERSION_MAJOR} EQUAL 3) AND (${Python_VERSION_MINOR} GREATER 10)))
        set(PY_INSTALL_ARGS "${PY_INSTALL_ARGS} --break-system-packages")
    endif()
endif()
install(CODE "
  execute_process(COMMAND ${Python_EXECUTABLE} -m pip install dist/moordyn-${MOORDYN_VERSION}.tar.gz ${PY_INSTALL_ARGS}
                  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
")
