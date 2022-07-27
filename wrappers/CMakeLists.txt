if(${PYTHON_WRAPPER})
    add_subdirectory(python)
endif()

if(${FORTRAN_WRAPPER})
    add_subdirectory(fortran)
endif()
