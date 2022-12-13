if(${FORTRAN_WRAPPER})
    add_subdirectory(fortran)
endif()

if(${MATLAB_WRAPPER})
    add_subdirectory(matlab)
endif()

if(${PYTHON_WRAPPER})
    add_subdirectory(python)
endif()

if(${RUST_WRAPPER})
    add_subdirectory(rust)
endif()
