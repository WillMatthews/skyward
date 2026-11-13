//! Settings › Video. STUB until the GFX track's version lands on `main`
//! (graphics presets and knobs): the orchestrator takes GFX's file.
//! Contract (agreed with the settings shell, ui/settings_ui.rs):
//! `pub fn video_tab(ui: &mut egui::Ui, st: &mut UiState) -> bool` draws the
//! category's rows into `ui` (a vertical layout inside the shell's scroll
//! area, full content width) and returns true when something changed. The
//! shell's row widgets are public in ui/settings_ui.rs (`rows_ui`) for a
//! consistent look; Apply / Revert / changed dots work on `st.settings`.

use super::UiState;

/// The Video category: until GFX lands, the display options that were in
/// the old Display tab (vertical sync, scenery detail).
pub fn video_tab(ui: &mut egui::Ui, st: &mut UiState) -> bool {
    let rows = super::settings_schema::rows(super::settings_schema::Cat::Video, false);
    super::settings_ui::rows_ui(ui, st, &rows)
}
