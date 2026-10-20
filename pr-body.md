# Description

Adds type-to-filter to the filament dropdowns. Opening a filament selector focuses a search field. Typing filters the list on every word, case-insensitively, matching the preset name, its alias and its group (vendor/type).

- **Keys:** Up/Down move through the results, Enter or a click selects, Escape closes without changing anything, and Tab moves on to the next control.
- **Selection safety:** filtering never changes which material is selected. Results keep the source item index, so ComboBox client data and preset IDs are untouched, and nothing is selected until the user accepts a row.
- **Favourites:** rows with an action icon (the favourite star in AMS material settings) still show and handle it. Clicking the icon fires `EVT_DROPDOWN_ITEM_ACTION` with the source index and leaves the popup open.
- **Opt-in:** `ComboBox::EnableTypeToFilter()` (`DropDown::EnableSearch()`) turns it on. Dropdowns that don't call it are unchanged.

Enabled for: the sidebar/preset filament combos (`PresetComboBoxes`), AMS material settings, calibration history, extrusion calibration, Create Filament Preset (vendor, type, preset and selected-filament combos), the per-object extruder column editor, the IMEX filament picker and the mixed-filament dialog. The numbered filament-slot selectors (per-object extruder, IMEX) use the slot's preset name as the alias, so "PLA" finds the slot loaded with it.

Implementation notes:
- The search mode is a `wxVListBox` inside the existing `DropDown` popup (`PopupWindow` with `wxPU_CONTAINS_CONTROLS`). The field is a `TextInput` that gets focus after `Popup()`.
- **On popup rule 18** (typing-focus content should use a frameless `wxDialog`): this is a deliberate exception. The search lives inside `DropDown` so that every existing `::ComboBox` gets it through one opt-in call, with the same anchoring, dismissal and Wayland/GTK chain handling it already has. Moving it to a dialog would fork that behaviour for search-enabled combos only. `SearchDialog` in `Search.cpp` already uses this structure for the options search: `PopupWindow(wxBORDER_NONE | wxPU_CONTAINS_CONTROLS)`, a `TextInput`, and `SetFocus()` after `Popup()`. On GTK, `ShouldDismissOnTopWindowDeactivate()` keeps the popup open while focus is in the search field or list.
- Colours come from the dropdown's own palette (`text_color`, `selector_background_color` Checked) through `StateColor`, re-read on every popup.
- Row clicks and action clicks run via `CallAfter`, because the selection and action handlers may rebuild the item list.
- Strings reuse the existing msgids "Type to filter..." and "No matching items...", so there are no new translations.
- **ExtrusionCalibration on macOS:** the filament selector now uses `::ComboBox` on macOS too, instead of a native `wxComboBox`; the native control cannot host the search. Every API the dialog calls on it is already used on Windows/Linux with the same type. The nozzle and bed selectors keep their macOS branch.

# Screenshots/Recordings/Graphs

These come from a Linux harness built from the actual `DropDown`, `TextInput`, `StaticBox` and palette code. Only resources are stubbed, so these are not full-application captures. The rows are representative presets, and the top line is the closed combo the popup opens from.

| Opened | "pla" | "bambu petg" (multi-word) |
|---|---|---|
| ![popup open, light](https://raw.githubusercontent.com/thereprocase/OrcaSlicer-nest3d/a46adb976/1-popup-open-light.png) | ![filter pla](https://raw.githubusercontent.com/thereprocase/OrcaSlicer-nest3d/a46adb976/2-filter-pla-light.png) | ![multi-word](https://raw.githubusercontent.com/thereprocase/OrcaSlicer-nest3d/a46adb976/3-multiword-bambu-petg-light.png) |

| No match | Dark theme |
|---|---|
| ![no match](https://raw.githubusercontent.com/thereprocase/OrcaSlicer-nest3d/a46adb976/4-no-match-light.png) | ![popup open, dark](https://raw.githubusercontent.com/thereprocase/OrcaSlicer-nest3d/a46adb976/5-popup-open-dark.png) |



## Tests

- [x] Linux (GCC Release): all 9 changed translation units compile; `git diff --check` clean; both reused msgids present in `OrcaSlicer.pot`; changed-line clang-tidy clean (local LLVM 23.1.1).
- [x] Linux (GTK3, X11 under Xvfb): an isolated harness built from the production `DropDown`, `TextInput` and `StaticBox` sources, with stub resources and no full app or printer setup. Passes: real keyboard typing; arrows; no-match; Escape; reopen. Also real mouse clicks: the favourite star keeps the popup open and selection unchanged and reports the source index, and clicking the highlighted row accepts exactly once. Run with both the light and the dark palette.
- [ ] Linux full application
- [ ] Tab traversal
- [ ] Linux Wayland
- [x] Windows (MSVC Release, full application): builds and links with no errors; launches with an isolated data directory.
- [ ] Windows: search interaction in the running app
- [ ] macOS: not run, as I have no Mac. CI covers the build. Help wanted from a macOS reviewer, especially on focus in the popup and the extrusion-calibration filament selector, which now uses `::ComboBox` on macOS.
- [ ] Light and dark theme, 100% and 150%+ scaling

🤖 Generated with [Claude Code](https://claude.com/claude-code)
