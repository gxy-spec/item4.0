# Frozen historical baseline configuration

This directory preserves the exact configurations used by the completed v2
36-run baseline.  Its B/C policies use the historical `hold_after_grace`
behavior, which is intentionally rejected by the current online-control safety
validator.  Keep these files for provenance and post-run analysis; do not use
them as templates for new simulator runs.

New experiments must use either bounded `coast_then_patrol` or
`persistent_target_memory`, retain UGV RGB-D obstacle guarding, and create a
new versioned configuration directory rather than modifying this frozen set.
