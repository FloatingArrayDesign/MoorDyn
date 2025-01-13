add_custom_target(
    rust_bindings
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml" "${CMAKE_CURRENT_BINARY_DIR}/Cargo.toml"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/lib.rs" "${CMAKE_CURRENT_BINARY_DIR}/lib.rs"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/build.rs" "${CMAKE_CURRENT_BINARY_DIR}/build.rs"
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/target/release/deps
    COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_LINKER_FILE:moordyn> ${CMAKE_CURRENT_BINARY_DIR}/target/release/deps/$<TARGET_LINKER_FILE_NAME:moordyn>
    COMMAND bindgen ${CMAKE_SOURCE_DIR}/source/MoorDyn2.h -o ${CMAKE_CURRENT_BINARY_DIR}/bindings.rs
)

add_custom_target(
    rust_wrapper ALL
    COMMAND cargo build --release
)
add_dependencies(rust_wrapper rust_bindings moordyn)
