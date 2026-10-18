# Patch the active CMake template instead of freezing a copy of its installer.
# The guarded replacements fail configuration if CMake changes these lifecycle points.
set(REPRO_TRUST_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../../scripts/windows/preserve_printer_trust.ps1")
set(REPRO_TRUST_LEGACY "${CMAKE_CURRENT_LIST_DIR}/legacy-printer-stock.sha256")
file(READ "${CMAKE_ROOT}/Modules/Internal/CPack/NSIS.template.in" _repro_nsis)
file(READ "${CMAKE_CURRENT_LIST_DIR}/PrinterTrust.nsh.in" _repro_trust)
string(CONFIGURE "${_repro_trust}" _repro_trust @ONLY)

function(repro_insert_before needle addition)
    string(FIND "${_repro_nsis}" "${needle}" _repro_position)
    if(_repro_position EQUAL -1)
        message(FATAL_ERROR "Repro printer trust: CMake NSIS lifecycle anchor changed: ${needle}")
    endif()
    string(REPLACE "${needle}" "${addition}\n${needle}" _repro_nsis "${_repro_nsis}")
    set(_repro_nsis "${_repro_nsis}" PARENT_SCOPE)
endfunction()

repro_insert_before("Function .onInit" "${_repro_trust}")
# $3 is the OLD install folder, before the previous uninstaller removes its files.
repro_insert_before([=[  ExecWait '"$0" /S _?=$3']=] [=[  Push $3
  Call ReproCapturePrinterTrust
  ClearErrors]=])
# Also handle an in-place repair when the owner declines the old-version uninstall.
repro_insert_before("  @CPACK_NSIS_EXTRA_PREINSTALL_COMMANDS@" [=[  Push $INSTDIR
  Call ReproCapturePrinterTrust]=])
repro_insert_before("@CPACK_NSIS_EXTRA_INSTALL_COMMANDS@" "  Call ReproRestorePrinterTrust")

set(_repro_template_dir "${CMAKE_CURRENT_BINARY_DIR}/repro-cpack")
file(MAKE_DIRECTORY "${_repro_template_dir}")
file(WRITE "${_repro_template_dir}/NSIS.template.in" "${_repro_nsis}")
list(PREPEND CMAKE_MODULE_PATH "${_repro_template_dir}")
install(FILES "${CMAKE_SOURCE_DIR}/resources/cert/printer.cer"
        DESTINATION "resources/cert" RENAME "printer-stock.cer")
