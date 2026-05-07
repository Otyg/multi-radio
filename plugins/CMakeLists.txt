cmake_minimum_required(VERSION 3.20)

set(MR_PLUGIN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/plugins")
file(MAKE_DIRECTORY "${MR_PLUGIN_OUTPUT_DIR}")

# Detect libliquid (same logic as backend/CMakeLists.txt).
find_package(PkgConfig QUIET)
set(MR_PLUGIN_LIQUID_TARGET "")
if(PkgConfig_FOUND)
  pkg_check_modules(LIQUID QUIET IMPORTED_TARGET liquid)
  if(LIQUID_FOUND)
    set(MR_PLUGIN_LIQUID_TARGET PkgConfig::LIQUID)
  endif()
endif()
if(NOT MR_PLUGIN_LIQUID_TARGET)
  find_path(MR_LIQUID_INC liquid/liquid.h PATHS /usr/local/include /usr/include)
  find_library(MR_LIQUID_LIB NAMES liquid PATHS /usr/local/lib /usr/lib /usr/lib64)
  if(MR_LIQUID_INC AND MR_LIQUID_LIB)
    add_library(_plugin_liquid INTERFACE)
    target_include_directories(_plugin_liquid INTERFACE ${MR_LIQUID_INC})
    target_link_libraries(_plugin_liquid INTERFACE ${MR_LIQUID_LIB})
    set(MR_PLUGIN_LIQUID_TARGET _plugin_liquid)
  endif()
endif()

function(mr_add_plugin target source)
  add_library(${target} MODULE ${source})
  set_target_properties(${target} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${MR_PLUGIN_OUTPUT_DIR}"
  )
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${target} PRIVATE m)
  set_property(TARGET ${target} PROPERTY C_STANDARD 99)
  set_property(TARGET ${target} PROPERTY C_STANDARD_REQUIRED ON)
  if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(${target} PRIVATE
      -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/plugin_exports.ld
    )
  endif()
  if(MR_PLUGIN_LIQUID_TARGET)
    target_link_libraries(${target} PRIVATE ${MR_PLUGIN_LIQUID_TARGET})
    target_compile_definitions(${target} PRIVATE MR_PLUGIN_HAS_LIQUID=1)
  endif()
  install(TARGETS ${target} LIBRARY DESTINATION "${MR_PLUGIN_OUTPUT_DIR}")
endfunction()

# ---------------------------------------------------------------------------
# Write a linker version script that exports only the four mandatory symbols.
# ---------------------------------------------------------------------------
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/plugin_exports.ld"
"{ global: mr_plugin_create; mr_plugin_destroy; mr_plugin_get_meta; mr_plugin_process_iq; mr_plugin_process_bits; mr_plugin_set_param; local: *; };\n"
)

# ---------------------------------------------------------------------------
# Plugin targets
# ---------------------------------------------------------------------------

mr_add_plugin(fsk_demod      fsk_demod.c)
mr_add_plugin(gmsk_demod     gmsk_demod.c)
mr_add_plugin(nrzi_decoder   nrzi_decoder.c)
mr_add_plugin(hdlc_postproc  hdlc_postproc.c)
mr_add_plugin(ais_decoder    ais_decoder.c)
