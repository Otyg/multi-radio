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

if(NOT MR_PLUGIN_LIQUID_TARGET)
  message(FATAL_ERROR "libliquid is required to build plugins. Install liquid-dsp development files.")
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
  target_link_libraries(${target} PRIVATE ${MR_PLUGIN_LIQUID_TARGET})
  target_compile_definitions(${target} PRIVATE MR_PLUGIN_HAS_LIQUID=1)
  install(TARGETS ${target} LIBRARY DESTINATION "backend/plugins")
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
mr_add_plugin(ppm_demod      ppm_demod.c)
mr_add_plugin(nrzi_decoder   nrzi_decoder.c)
mr_add_plugin(hdlc_postproc  hdlc_postproc.c)
mr_add_plugin(ais_decoder     ais_decoder.c)
mr_add_plugin(asm_decoder     asm_decoder.c)
mr_add_plugin(ais_dual_demod  ais_dual_demod.c)
mr_add_plugin(iq_recorder iq_recorder.c)
mr_add_plugin(vdes_phy_demod   vdes_phy_demod.c)
mr_add_plugin(vdes_asm_demod   vdes_asm_demod.c)
mr_add_plugin(vdes_asm_decoder vdes_asm_decoder.c)
mr_add_plugin(vdes_asm_postproc vdes_asm_postproc.c)
mr_add_plugin(vdes_burst_decoder vdes_burst_decoder.c)
mr_add_plugin(vdes_burst_demod   vdes_burst_demod.c)
mr_add_plugin(dsc_afsk_demod     dsc_afsk_demod.c)
mr_add_plugin(dsc_decoder        dsc_decoder.c)

# ---------------------------------------------------------------------------
# adsb_demod: ADS-B demodulator backed by libmodes (dump1090)
# ---------------------------------------------------------------------------
set(LIBMODES_LIB "")
set(LIBMODES_INC "")

if(DEFINED MR_LIBMODES_ARCHIVE AND NOT "${MR_LIBMODES_ARCHIVE}" STREQUAL "")
  set(LIBMODES_LIB "${MR_LIBMODES_ARCHIVE}")
endif()

if(DEFINED MR_LIBMODES_INCLUDE_DIR AND NOT "${MR_LIBMODES_INCLUDE_DIR}" STREQUAL "")
  set(LIBMODES_INC "${MR_LIBMODES_INCLUDE_DIR}")
endif()

if((NOT LIBMODES_LIB OR NOT LIBMODES_INC) AND PkgConfig_FOUND)
  pkg_check_modules(LIBMODES QUIET IMPORTED_TARGET libmodes)
  if(LIBMODES_FOUND)
    if(NOT LIBMODES_LIB AND DEFINED LIBMODES_LINK_LIBRARIES AND NOT "${LIBMODES_LINK_LIBRARIES}" STREQUAL "")
      list(GET LIBMODES_LINK_LIBRARIES 0 LIBMODES_LIB)
    endif()
    if(NOT LIBMODES_INC AND DEFINED LIBMODES_INCLUDE_DIRS AND NOT "${LIBMODES_INCLUDE_DIRS}" STREQUAL "")
      list(GET LIBMODES_INCLUDE_DIRS 0 LIBMODES_INC)
    endif()
  endif()
endif()

if(NOT LIBMODES_LIB)
  find_library(LIBMODES_LIB NAMES modes libmodes
    PATHS
      /usr/local/lib
      /usr/lib
      "$ENV{HOME}/projects/dump1090"
  )
endif()

if(NOT LIBMODES_INC)
  find_path(LIBMODES_INC libmodes/libmodes.h
    PATHS
      /usr/local/include
      /usr/include
  )
endif()

if(LIBMODES_LIB AND LIBMODES_INC)
  mr_add_plugin(adsb_demod adsb_demod.c)
  target_include_directories(adsb_demod PRIVATE ${LIBMODES_INC})
  target_link_libraries(adsb_demod PRIVATE ${LIBMODES_LIB})
  message(STATUS "adsb_demod: using libmodes from ${LIBMODES_LIB}")
else()
  message(FATAL_ERROR "libmodes not found — cannot build adsb_demod. Install libmodes or check /usr/local/lib.")
endif()
