# Repro Windows installer: local printer trust

`ORCA_REPRO_PRESERVE_LOCAL_TRUST` enables certificate migration in the Windows
NSIS installer. It defaults to OFF. The build workflow enables it only for
`thereprocase/OrcaSlicer-nest3d`, branch `repro-dev`. Keep this fork-specific
change out of upstream submissions.

Before invoking the previous uninstaller, the new installer reads the old
installation's `resources/cert/printer.cer`. It extracts public X.509
certificates into the installer's temporary `$PLUGINSDIR`. An in-place repair
captures the destination bundle before copying new files instead. This also
works for silent `/S` upgrades; no user-profile path is involved.

Each enabled build installs `printer-stock.cer`, an exact pristine copy of that
build's vendor bundle. The migration subtracts the **old** baseline and the
checked-in legacy vendor fingerprints from the old installed trust set. It
appends only the remaining certificates to the **new** stock bundle, deduplicated
by SHA-256 of DER. This preserves local bridge trust while allowing vendor
certificates to be removed in updates. The legacy fingerprints cover the vendor
bundle predating this feature and contain no owner-specific material.

The operation reads certificate files only. Access codes and printer profiles
remain in Orca's existing configuration. No bridge address, certificate, private
key or access code is embedded in the repository, installer or CI artifacts.
TLS verification remains enabled. Migration preserves the trust already present
in the selected installation; the installer does not discover or fetch new trust.

Capture errors abort before uninstalling. Restore validates the entire output
and atomically replaces the freshly installed bundle. On restore failure, stock
trust remains intact and `printer-local-recovery.cer` retains the captured public
certificates for recovery. The installer reports failure; it does not silently
claim the bridge is ready. Close Orca before upgrading.

## Boundaries

- The first update requires bridge trust to be present in the current installation.
  If an earlier update already removed it, run bridge setup once before upgrading.
- Normal upgrades, repeat installs, directory changes through the registered old
  uninstaller, and upgrades to future enabled builds are covered.
- Portable ZIP extraction, MSIX, upstream installers and older installers without
  this feature do not run this migration. They may require bridge setup again.
- Plain uninstall removes the installed bundle and its baseline; the installer
  does not create a permanent per-user trust store.
- Removing a local certificate from `printer.cer` revokes it for the next update;
  the migration does not resurrect a separate remembered copy.

## Validation

Run `tests/packaging/test_preserve_printer_trust.ps1` with Windows PowerShell 5.1.
It uses isolated temporary folders and existing public vendor certificates as
fixtures. It covers vendor retirement, legacy migration, repeat upgrade, repair,
downgrade, duplicate/rewrapped certificates, malformed input, locked output and
byte-identical stock-only installs. No live printer or real Orca install is used.

The CMake extension inserts capture before CPack's old-uninstaller call, plus
repair capture before file installation and restore afterward. Missing lifecycle
anchors fail configuration so a CMake template change cannot silently remove
the preservation step. Template handling follows CPack's
[NSIS packaging extension mechanism](https://cmake.org/cmake/help/book/mastering-cmake/chapter/Packaging%20With%20CPack.html).
